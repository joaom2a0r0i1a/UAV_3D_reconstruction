#!/usr/bin/env python

# Python
import sys
import time
import csv
import datetime
import os
import re
import subprocess

# ros
import rospy
import rosnode
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from voxblox_msgs.srv import FilePath
from mrs_msgs.msg import HwApiStatus


class EvalData(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_planner = rospy.get_param('~ns_planner',
                                          "/uav1/planner_node")
        self.planner_delay = rospy.get_param(
            '~delay', 0.0)  # Waiting time until the planner is launched
        self.evaluate = rospy.get_param(
            '~evaluate', False)  # Periodically save the voxblox state
        self.startup_timeout = rospy.get_param(
            '~startup_timeout', 0.0)  # Max allowed time for startup, 0 for inf

        self.eval_frequency = rospy.get_param('~eval_frequency',
                                              5.0)  # Save rate in seconds
        self.time_limit = rospy.get_param(
            '~time_limit', 0.0)  # Maximum sim duration in minutes, 0 for inf
        self.reset_ros = rospy.get_param(
            '~reset_ros', True)  # On shutdown stops the planner

        # Early stop: after the planner self-terminates, wait a grace then pad coverage to time_limit and stop (gated, default off).
        self.early_stop = rospy.get_param('~early_stop', False)
        self.early_stop_grace = rospy.get_param('~early_stop_grace',
                                                60.0)  # seconds after planner death
        self.planner_seen = False      # latch: only watch for death after it was alive
        self.grace_deadline = None     # ros-time at which to stop, once planner is gone

        self.eval_walltime_0 = None
        self.eval_rostime_0 = None

        if self.evaluate:
            # Setup parameters
            self.eval_directory = rospy.get_param(
                '~eval_directory',
                'DirParamNotSet')  # Periodically save voxblox map
            if not os.path.isdir(self.eval_directory):
                rospy.logfatal("Invalid target directory '%s'.",
                               self.eval_directory)
                sys.exit(-1)

            self.ns_voxblox = rospy.get_param('~ns_voxblox',
                                              "/uav1/voxblox_node")

            # Statistics
            self.eval_n_maps = 0
            self.eval_n_pointclouds = 0

            # Setup data directory
            if not os.path.isdir(os.path.join(self.eval_directory,
                                              "tmp_bags")):
                os.mkdir(os.path.join(self.eval_directory, "tmp_bags"))
            self.eval_directory = os.path.join(
                self.eval_directory,
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.mkdir(self.eval_directory)
            rospy.set_param(self.ns_planner + "/performance_log_dir",
                            self.eval_directory)
            os.mkdir(os.path.join(self.eval_directory, "voxblox_maps"))
            self.eval_data_file = open(
                os.path.join(self.eval_directory, "voxblox_data.csv"), 'w')
            self.eval_writer = csv.writer(self.eval_data_file,
                                          delimiter=',',
                                          quotechar='|',
                                          quoting=csv.QUOTE_MINIMAL,
                                          lineterminator='\n')
            self.eval_writer.writerow(
                ['MapName', 'RosTime', 'WallTime', 'NPointclouds'])#, 'CPUTime'])
            self.eval_writer.writerow(
                ['Unit', 'seconds', 'seconds', '-'])#, 'seconds'])
            self.eval_log_file = open(
                os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Subscribers, Services
            self.points_sub = rospy.Subscriber("points",
                                               PointCloud2,
                                               self.points_callback,
                                               queue_size=10)

            # Crash detection: unexpected disarm after /start (not takeoff, not teardown) => wall hit.
            self.armed = True
            self.crash_check_active = False
            self.hw_status_sub = rospy.Subscriber("hw_api/status", HwApiStatus,
                                                  self.hw_status_callback,
                                                  queue_size=1)
            self.crash_timer = rospy.Timer(rospy.Duration(1.0),
                                           self.crash_check_callback)

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("Data folder created at '%s'." % self.eval_directory)
            self.eval_voxblox_service = rospy.ServiceProxy(
                self.ns_voxblox + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)
            #self.collided = False

        self.launch_simulation()

    def launch_simulation(self):
        rospy.loginfo(
            "Experiment setup: waiting for MAV simulation to setup...")
        # Wait for simulation to setup
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_message("simulation_ready", Bool,
                                       self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment(
                    "Simulation startup failed (timeout after " +
                    str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_message("simulation_ready", Bool)
        rospy.loginfo("Waiting for MAV simulation to setup... done.")

        # Launch planner (by service, every planner needs to advertise this
        # service when ready)
        rospy.loginfo("Waiting for planner to be ready...")
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_service(self.ns_planner + "/start",
                                       self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment("Planner startup failed (timeout after " +
                                     str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_service(self.ns_planner + "/start")

        if self.planner_delay > 0:
            rospy.loginfo(
                "Waiting for planner to be ready... done. Launch in %d "
                "seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo("Waiting for planner to be ready... done.")
        run_planner_srv = rospy.ServiceProxy(
            self.ns_planner + "/start", Trigger)
        run_planner_srv()

        # Planner is now exploring -> arm the crash detector.
        self.crash_check_active = True

        # Setup first measurements
        self.eval_walltime_0 = time.time()
        self.eval_rostime_0 = rospy.get_time()
        # Evaluation init
        if self.evaluate:
            self.writelog("Succesfully started the simulation.")

            # Dump complete rosparams for reference
            subprocess.check_call([
                "rosparam", "dump",
                os.path.join(self.eval_directory, "rosparams.yaml"), "/"
            ])
            self.writelog("Dumped the parameter server into 'rosparams.yaml'.")

            self.eval_n_maps = 0
            self.eval_n_pointclouds = 1

            # Register the most recent rosbag; poll ~20s since the recorder may not have created it yet.
            bag_expr = re.compile(
                r'tmp_bag_\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}\.bag.')
            tmp_bags_dir = os.path.join(os.path.dirname(self.eval_directory),
                                        "tmp_bags")
            bags = []
            for _ in range(40):  # up to ~20 s for the recorder to create the .active bag
                bags = sorted(
                    [b for b in os.listdir(tmp_bags_dir) if bag_expr.match(b)],
                    reverse=True)
                if bags:
                    break
                rospy.sleep(0.5)
            if bags:
                self.writelog("Registered '%s' as bag for this simulation." %
                              bags[0])
                self.eval_log_file.write("[FLAG] Rosbag: %s\n" %
                                         bags[0].split('.')[0])
            else:
                rospy.logwarn("No tmpbag found after ~20 s. Is rosbag recording?")

        # Periodic evaluation (call once for initial measurement)
        self.eval_callback(None)
        rospy.Timer(rospy.Duration(self.eval_frequency), self.eval_callback)

        # Finish
        rospy.loginfo("\n" + "*" * 39 +
                      "\n* Succesfully started the simulation! *\n" + "*" * 39)

    def eval_callback(self, _):
        if self.evaluate:
            # Produce a data point
            time_real = time.time() - self.eval_walltime_0
            time_ros = rospy.get_time() - self.eval_rostime_0
            map_name = "{0:05d}".format(self.eval_n_maps)
            self.eval_writer.writerow([
                map_name, time_ros, time_real, self.eval_n_pointclouds#,
                #float(cpu.message)
            ])
            self.eval_voxblox_service(
                os.path.join(self.eval_directory, "voxblox_maps",
                             map_name + ".vxblx"))
            self.eval_n_pointclouds = 0
            self.eval_n_maps += 1

        # Early stop: planner self-shuts-down when done; wait a grace, pad the curve to time_limit, finalize the bag, stop.
        if self.early_stop and self.time_limit > 0.0:
            if self._planner_alive():
                self.planner_seen = True
            elif self.planner_seen:
                if self.grace_deadline is None:
                    self.writelog(
                        "Planner terminated; %ds grace before stop." %
                        int(self.early_stop_grace))
                    rospy.loginfo(
                        "[eval] Planner terminated; %ds grace before stop." %
                        int(self.early_stop_grace))
                    self.grace_deadline = rospy.get_time() + self.early_stop_grace
                # 'if' (not 'elif') so grace=0 stops on the same tick termination is detected (map already saved above; no coverage lost).
                if rospy.get_time() >= self.grace_deadline:
                    self._pad_to_time_limit()
                    self._finalize_rosbag()
                    self.stop_experiment(
                        "Planner terminated; padded curve to time_limit.")
                    return

        # If the time limit is reached stop the simulation
        if self.time_limit > 0.0:
            if rospy.get_time(
            ) - self.eval_rostime_0 >= self.time_limit * 60.0:
                self.stop_experiment("Time limit reached.")

    def _planner_alive(self):
        # Ping the planner node; False once it self-terminates (ros::shutdown).
        full = self.ns_planner if self.ns_planner.startswith('/') \
            else (rospy.get_namespace() + self.ns_planner)
        try:
            return rosnode.rosnode_ping(full, max_count=1, verbose=False)
        except Exception:
            return False

    def _pad_to_time_limit(self):
        # Hold the final map's coverage flat out to time_limit so the curve spans the full budget (reuses the final MapName at rising RosTime).
        if not self.evaluate:
            return
        last_map = "{0:05d}".format(max(self.eval_n_maps - 1, 0))
        t = rospy.get_time() - self.eval_rostime_0
        wt = time.time() - self.eval_walltime_0
        target = self.time_limit * 60.0
        pad_rows = 0
        # Repeat the final map's coverage at rising t until the curve spans the full budget.
        while t < target:
            t += self.eval_frequency
            self.eval_writer.writerow([last_map, t, wt, 0])
            pad_rows += 1
        self.eval_data_file.flush()
        self.writelog(
            "Padded curve with final map %s: +%d rows up to %.0f min." %
            (last_map, pad_rows, self.time_limit))

    def _finalize_rosbag(self):
        # Clean-kill the recorder so it renames tmp_bag_*.bag.active -> .bag
        # (a SIGKILL from teardown would leave an unusable .active).
        try:
            os.system("rosnode kill /uav1/rosbag_recorder >/dev/null 2>&1")
            rospy.sleep(3.0)  # allow the rename to complete
        except Exception:
            pass

    def eval_finish(self):
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "voxblox_maps")
        n_maps = len([
            f for f in os.listdir(map_path)
            if os.path.isfile(os.path.join(map_path, f))
        ])
        self.writelog("Finished the simulation, %d/%d maps created." %
                      (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo("On eval_data_node shutdown: closing data files.")
        # Sentinel written last (after CSV closed): signals run_experiments.sh the run finished cleanly.
        try:
            open(os.path.join(os.path.dirname(self.eval_directory),
                              ".run_complete"), "w").close()
        except Exception:
            pass

    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(
            datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text +
            "\n")

    def points_callback(self, _):
        if self.evaluate:
            self.eval_n_pointclouds += 1

    def hw_status_callback(self, msg):
        self.armed = msg.armed

    def crash_check_callback(self, event):
        # Unexpected disarm between /start and stop => wall hit: drop '.crashed' (supervisor retries) and stop.
        if not self.crash_check_active or self.armed:
            return
        self.crash_check_active = False
        try:
            open(os.path.join(self.eval_directory, ".crashed"), "w").close()
        except Exception:
            pass
        self.writelog("Disarmed mid-run (likely wall hit) -> run flagged '.crashed'.")
        rospy.logwarn("[eval] Disarmed mid-run (likely wall hit) -> discarding run.")
        self.stop_experiment("Disarmed mid-run (likely wall hit).")

    def stop_experiment(self, reason):
        # Shutdown the node with proper logging, only required when experiment
        # is performed
        self.crash_check_active = False  # any stop disables the detector (teardown disarm is expected)
        reason = "Stopping the experiment: " + reason
        if self.evaluate:
            self.writelog(reason)
        if self.reset_ros:
            try:
                # Stops the planner
                terminate_srv = rospy.ServiceProxy(
                    self.ns_planner + "/stop", Trigger)
                terminate_srv()
            except:
                pass
        width = len(reason) + 4
        rospy.loginfo("\n" + "*" * width + "\n* " + reason + " *\n" +
                      "*" * width)
        rospy.signal_shutdown(reason)



if __name__ == '__main__':
    rospy.init_node('eval_data_node', anonymous=True)
    ed = EvalData()
    rospy.spin()
