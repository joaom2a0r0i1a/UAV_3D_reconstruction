#!/usr/bin/env python3
# Interactive planner start gate. Confirm after GUIDED; then s stops, o re-offsets, q quits.
import select
import sys

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class StartGate(object):
    def __init__(self):
        self.planner_ns = rospy.get_param('~ns_planner', 'planner_node')
        self.mode_required = rospy.get_param('~mode_required', 'GUIDED')

        # automatic starts the planner itself once the preconditions hold, manual asks first
        self.auto_start = rospy.get_param('~auto_start', False)
        
        # preconditions must hold this long before an automatic start, mavros mode and armed
        # both glitch for single samples and must not trigger a mission
        self.auto_start_settle = rospy.get_param('~auto_start_settle', 3.0)
        self.mode = '?'
        self.armed = False
        self.last_pose_t = None
        self.ready_since = None
        self.planner_up_cached = False
        self.planner_checked_t = None
        self.planner_check_period = rospy.get_param('~planner_check_period', 2.0)
        rospy.Subscriber('state_in', State, self.cb_state, queue_size=1)
        rospy.Subscriber('pose_in', PoseStamped, self.cb_pose, queue_size=1)
        self.srv_start = rospy.ServiceProxy(self.planner_ns + '/start', Trigger)
        self.srv_stop = rospy.ServiceProxy(self.planner_ns + '/stop', Trigger)
        self.srv_offset = rospy.ServiceProxy(self.planner_ns + '/offset', Trigger)

    def cb_state(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed

    def cb_pose(self, msg):
        self.last_pose_t = rospy.get_time()

    def pose_fresh(self):
        return (self.last_pose_t is not None
                and rospy.get_time() - self.last_pose_t < 2.0)

    def planner_up(self):
        # throttled: the probe is an XML-RPC round trip, no need to run it every refresh
        now = rospy.get_time()
        if (self.planner_checked_t is not None
                and now - self.planner_checked_t < self.planner_check_period):
            return self.planner_up_cached
        try:
            rospy.wait_for_service(self.planner_ns + '/start', timeout=0.2)
            self.planner_up_cached = True
        except rospy.ROSException:
            self.planner_up_cached = False
        self.planner_checked_t = now
        return self.planner_up_cached

    def status_line(self):
        return ("mode=%-8s armed=%s planner=%s pose=%s      " %
                (self.mode, "Y" if self.armed else "n",
                 "up" if self.planner_up() else "DOWN",
                 "fresh" if self.pose_fresh() else "STALE"))

    def prompt(self, text):
        # Full-line prompt with bell; blocks THIS pane only.
        sys.stdout.write('\a\n' + text)
        sys.stdout.flush()
        return sys.stdin.readline().strip().lower()

    def wait_key(self, timeout):
        # Non-blocking single-line read with timeout (lets the status line keep refreshing).
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.readline().strip().lower()
        return None

    def run(self):
        rospy.loginfo("[start_gate]: up — waiting for %s mode (planner ns: %s)",
                      self.mode_required, self.planner_ns)
        started = False
        while not rospy.is_shutdown():
            if not started:
                sys.stdout.write('\r' + self.status_line())
                sys.stdout.flush()
                ready = (self.mode == self.mode_required and self.armed
                         and self.planner_up() and self.pose_fresh())
                if not ready:
                    self.ready_since = None
                    rospy.sleep(1.0)
                    continue
                if self.auto_start:
                    if self.ready_since is None:
                        self.ready_since = rospy.get_time()
                    if rospy.get_time() - self.ready_since < self.auto_start_settle:
                        rospy.sleep(0.5)
                        continue
                    ans = 'y'
                    print("\nauto start: preconditions held for %.0fs, starting planner."
                          % self.auto_start_settle)
                else:
                    ans = self.prompt("Start Planner? [Y/n] ")
                if ans in ('', 'y', 'yes'):
                    try:
                        res = self.srv_start()
                        print("start -> success=%s: %s" % (res.success, res.message))
                        started = bool(res.success)
                    except rospy.ServiceException as e:
                        print("start FAILED: %s" % e)
                else:
                    print("Not starting; will ask again in 10 s (Ctrl-C to quit).")
                    rospy.sleep(10.0)
            else:
                sys.stdout.write('\r' + self.status_line() +
                                 "| [s]top [o]ffset re-capture [q]uit: ")
                sys.stdout.flush()
                key = self.wait_key(2.0)
                if key is None:
                    continue
                if key == 's':
                    try:
                        res = self.srv_stop()
                        print("\nstop -> success=%s: %s" % (res.success, res.message))
                    except rospy.ServiceException as e:
                        print("\nstop FAILED (planner busy planning? kill the "
                              "node if needed): %s" % e)
                elif key == 'o':
                    try:
                        res = self.srv_offset()
                        print("\noffset -> success=%s: %s" % (res.success, res.message))
                    except rospy.ServiceException as e:
                        print("\noffset FAILED: %s" % e)
                elif key == 'q':
                    print("\nbye")
                    return


if __name__ == '__main__':
    rospy.init_node('start_gate', anonymous=True)
    StartGate().run()
