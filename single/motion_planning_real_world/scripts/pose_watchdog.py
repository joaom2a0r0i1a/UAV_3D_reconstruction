#!/usr/bin/env python
# Warns when /mavros/local_position/pose goes quiet. A >5 s gap is what latches the TF
# broadcaster's guard and takes voxblox down with it, so make gaps visible live.
import rospy
from geometry_msgs.msg import PoseStamped

class PoseWatchdog(object):
    def __init__(self):
        self.warn_gap = rospy.get_param('~warn_gap', 1.0)
        self.report_every = rospy.get_param('~report_every', 30.0)
        self.last = None
        self.max_gap = 0.0
        self.n_gaps = 0
        self.n_msgs = 0
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cb, queue_size=10)
        rospy.Timer(rospy.Duration(self.report_every), self.report)
        rospy.Timer(rospy.Duration(0.5), self.check_stale)

    def cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.n_msgs += 1
        if self.last is not None:
            gap = now - self.last
            if gap > self.warn_gap:
                self.n_gaps += 1
                rospy.logwarn('[pose_watchdog]: GAP %.2f s (gap #%d) — >5 s latches the TF guard', gap, self.n_gaps)
            if gap > self.max_gap:
                self.max_gap = gap
        self.last = now

    def check_stale(self, _):
        if self.last is None:
            return
        stale = rospy.Time.now().to_sec() - self.last
        if stale > self.warn_gap:
            rospy.logwarn_throttle(2.0, '[pose_watchdog]: NO POSE for %.1f s', stale)

    def report(self, _):
        rospy.loginfo('[pose_watchdog]: %d msgs, %d gaps > %.1f s, worst %.2f s',
                      self.n_msgs, self.n_gaps, self.warn_gap, self.max_gap)

if __name__ == '__main__':
    rospy.init_node('pose_watchdog')
    PoseWatchdog()
    rospy.spin()
