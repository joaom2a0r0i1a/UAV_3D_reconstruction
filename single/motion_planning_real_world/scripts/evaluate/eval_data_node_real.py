#!/usr/bin/env python

# Real flight recorder. Starts on the latched offset_out, writes offset.txt for stage 2.

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
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from voxblox_msgs.srv import FilePath


class EvalData(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_planner = rospy.get_param('~ns_planner', "planner_node")
        self.evaluate = rospy.get_param(
            '~evaluate', False)  # Periodically save the voxblox state
        self.startup_timeout = rospy.get_param(
            '~startup_timeout', 0.0)  # Max allowed time for startup, 0 for inf

        self.eval_frequency = rospy.get_param('~eval_frequency',
                                              5.0)  # Save rate in seconds
        self.time_limit = rospy.get_param(
            '~time_limit', 0.0)  # Maximum mission duration in minutes, 0 for inf
        self.reset_ros = rospy.get_param(
            '~reset_ros', True)  # On shutdown stops the planner

        # Early stop: after the planner self-terminates, wait a grace then pad coverage to time_limit and stop.
        self.early_stop = rospy.get_param('~early_stop', True)
        self.early_stop_grace = rospy.get_param('~early_stop_grace', 10.0)
        self.planner_misses = 0        # consecutive failed pings, debounces a flaky ping
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

            self.ns_voxblox = rospy.get_param('~ns_voxblox', "voxblox_node")

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
                ['MapName', 'RosTime', 'WallTime', 'NPointclouds'])
            self.eval_writer.writerow(['Unit', 'seconds', 'seconds', '-'])
            self.eval_log_file = open(
                os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Subscribers, Services
            self.points_sub = rospy.Subscriber("points",
                                               PointCloud2,
                                               self.points_callback,
                                               queue_size=10)

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("Data folder created at '%s'." % self.eval_directory)
            self.eval_voxblox_service = rospy.ServiceProxy(
                self.ns_voxblox + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)

        self.launch_experiment()

    def launch_experiment(self):
        rospy.loginfo("Experiment setup: waiting for the planner stack...")
        # Wait for the planner to report readiness (fresh pose)
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_message("simulation_ready", Bool,
                                       self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment(
                    "Planner stack startup failed (timeout after " +
                    str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_message("simulation_ready", Bool)
        rospy.loginfo("Planner stack is up.")

        # REAL-WORLD DELTA: don't call the start service — the operator takes off + GUIDEs, and the planner's latched ~offset_out (published in captureOffset at ~start) is the mission-start trigger and t0.
        rospy.loginfo("Waiting for mission start (latched offset from the "
                      "planner; use start_gate.py or 'rosservice call "
                      "%s/start')...", self.ns_planner)
        offset_msg = rospy.wait_for_message("offset_in", Point)

        # Mission is starting NOW -> t0.
        self.eval_walltime_0 = time.time()
        self.eval_rostime_0 = rospy.get_time()

        # Evaluation init
        if self.evaluate:
            self.writelog("Mission started (offset received).")

            # Persist the run offset so stage-2 eval shifts the volume box by it (eval_voxblox_node auto-reads '<run>/offset.txt').
            with open(os.path.join(self.eval_directory, "offset.txt"),
                      'w') as f:
                f.write("%.6f %.6f %.6f\n" %
                        (offset_msg.x, offset_msg.y, offset_msg.z))
            self.eval_log_file.write("[FLAG] Offset: %.6f %.6f %.6f\n" %
                                     (offset_msg.x, offset_msg.y,
                                      offset_msg.z))
            self.writelog("Run offset: [%.2f, %.2f, %.2f]." %
                          (offset_msg.x, offset_msg.y, offset_msg.z))

            # Dump complete rosparams for reference
            try:
                subprocess.check_call([
                    "rosparam", "dump",
                    os.path.join(self.eval_directory, "rosparams.yaml"), "/"
                ])
                self.writelog("Dumped the parameter server into 'rosparams.yaml'.")
            except Exception as e:
                rospy.logwarn("rosparam dump failed (%s), continuing.", e)
                self.writelog("rosparam dump failed, continuing without it.")

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
                self.writelog("Registered '%s' as bag for this run." % bags[0])
                self.eval_log_file.write("[FLAG] Rosbag: %s\n" %
                                         bags[0].split('.')[0])
            else:
                rospy.logwarn(
                    "No tmpbag found after ~20 s. Is 'record.sh eval' running?")

        # Periodic evaluation (call once for initial measurement)
        self.eval_callback(None)
        rospy.Timer(rospy.Duration(self.eval_frequency), self.eval_callback)

        # Finish
        rospy.loginfo("\n" + "*" * 37 +
                      "\n* Mission recording is now active! *\n" + "*" * 37)

    def eval_callback(self, _):
        if self.evaluate:
            # Produce a data point
            time_real = time.time() - self.eval_walltime_0
            time_ros = rospy.get_time() - self.eval_rostime_0
            map_name = "{0:05d}".format(self.eval_n_maps)
            self.eval_writer.writerow(
                [map_name, time_ros, time_real, self.eval_n_pointclouds])
            self.eval_data_file.flush()   # survive a hard power off in the field
            try:
                self.eval_voxblox_service(
                    os.path.join(self.eval_directory, "voxblox_maps",
                                 map_name + ".vxblx"))
            except Exception as e:
                rospy.logwarn_throttle(10, "save_map failed (%s), continuing." % e)
            self.eval_n_pointclouds = 0
            self.eval_n_maps += 1

        # Early stop: planner self-shuts-down when done; wait a grace, pad the curve to time_limit, finalize the bag, stop.
        if self.early_stop and self.time_limit > 0.0:
            if self._planner_alive():
                self.planner_seen = True
                self.planner_misses = 0
                self.grace_deadline = None   # a recovered ping must not leave the stop armed
            elif self.planner_seen:
                self.planner_misses += 1
                if self.planner_misses < 3:   # ignore a single flaky ping
                    return
                if self.grace_deadline is None:
                    self.writelog(
                        "Planner terminated; %ds grace before stop." %
                        int(self.early_stop_grace))
                    rospy.loginfo(
                        "[eval] Planner terminated; %ds grace before stop." %
                        int(self.early_stop_grace))
                    self.grace_deadline = rospy.get_time() + self.early_stop_grace
                # 'if' (not 'elif') so grace=0 stops on the same tick termination is detected.
                if rospy.get_time() >= self.grace_deadline:
                    self._pad_to_time_limit()
                    self._finalize_rosbag()
                    self.stop_experiment(
                        "Planner terminated; padded curve to time_limit.")
                    return

        # If the time limit is reached stop the mission recording
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
        # Hold the final map's coverage flat out to time_limit so the curve spans the full budget.
        if not self.evaluate:
            return
        last_map = "{0:05d}".format(max(self.eval_n_maps - 1, 0))
        t = rospy.get_time() - self.eval_rostime_0
        wt = time.time() - self.eval_walltime_0
        target = self.time_limit * 60.0
        pad_rows = 0
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
        try:
            os.system("rosnode kill /eval_bag_recorder >/dev/null 2>&1")
            time.sleep(3.0)  # wall clock: rospy.sleep raises once shutdown started
        except Exception:
            pass

    def eval_finish(self):
        # Runs on every shutdown path, Ctrl+C included, so an aborted flight still
        # leaves a closed bag and a complete run folder.
        self._finalize_rosbag()
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "voxblox_maps")
        n_maps = len([
            f for f in os.listdir(map_path)
            if os.path.isfile(os.path.join(map_path, f))
        ])
        self.writelog("Finished the mission, %d/%d maps created." %
                      (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo("On eval_data_node shutdown: closing data files.")
        # Sentinel written last (after CSV closed): marks the run complete for offload_runs.sh.
        try:
            open(os.path.join(self.eval_directory, ".run_complete"),
                 "w").close()
        except Exception:
            pass

    def writelog(self, text):
        if not self.evaluate:
            return
        self.eval_log_file.write(
            datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text +
            "\n")
        self.eval_log_file.flush()

    def points_callback(self, _):
        if self.evaluate:
            self.eval_n_pointclouds += 1

    def stop_experiment(self, reason):
        reason = "Stopping the experiment: " + reason
        if self.evaluate:
            self.writelog(reason)
        if self.reset_ros:
            try:
                terminate_srv = rospy.ServiceProxy(
                    self.ns_planner + "/stop", Trigger)
                terminate_srv()
            except Exception:
                pass
        width = len(reason) + 4
        rospy.loginfo("\n" + "*" * width + "\n* " + reason + " *\n" +
                      "*" * width)
        rospy.signal_shutdown(reason)


if __name__ == '__main__':
    rospy.init_node('eval_data_node', anonymous=True)
    ed = EvalData()
    rospy.spin()
