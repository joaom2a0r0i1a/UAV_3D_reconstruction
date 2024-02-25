#!/usr/bin/python3

import rospy
import numpy
#import rrt_star
import rrt_star_2d

from mrs_msgs.msg import ControlManagerDiagnostics,Reference
from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.srv import Vec1,Vec1Response

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("motion_planner", anonymous=True)

        ## | --------------------- load parameters -------------------- |

        self.frame_id = rospy.get_param("~frame_id")

        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = rospy.get_param("~center/z")

        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")

        self.timer_main_rate = rospy.get_param("~timer_main/rate")

        rospy.loginfo('[motion_planner]: initialized')

        ## | ----------------------- subscribers ---------------------- |

        self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        ## | --------------------- service servers -------------------- |

        self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)

        ## | --------------------- service clients -------------------- |

        self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)

        ## | ------------------------- timers ------------------------- |

        self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rospy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    # #{ planPath()

    def planPath(self, step_size):

        rospy.loginfo('[motion_planner]: planning path')

        # Define start and goal points
        #start = [self.center_x, self.center_y, self.center_z]
        #goal = [self.center_x + 4, self.center_y + 4, self.center_z + 4]  # Example goal point

        start = [self.center_x, self.center_y]
        goal = [self.center_x+6, self.center_y+4]

        max_iterations = 1000
        #step_size = 0.3
        radius = 1

        dim_x = self.dimensions_x
        dim_y = self.dimensions_y
        #dim_z = self.dimensions_y

        # Define obstacles
        obstacles = []  # Fill in with obstacles if needed

        # Call RRT* algorithm to generate path
        #tree, path = rrt_star.rrt_star(start, goal, obstacles, dim_x, dim_y, dim_z, max_iter=max_iterations, step_size=step_size, radius=radius)
        tree, path = rrt_star_2d.rrt_star(start, goal, obstacles,  dim_x, dim_y, max_iter=max_iterations, step_size=step_size, radius=radius)
        
        # Convert path to a ROS message
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        #path_msg.path.stop_at_waypoints = True

        for point in path:
            reference = Reference()
            reference.position.x = point[0]
            reference.position.y = point[1]
            #reference.position.z = point[2] # For 3D
            reference.position.z = self.center_z # For 2D
            reference.heading = 0.0  # Adjust heading if needed
            path_msg.path.points.append(reference)
            #print(f"Reference: {reference}, Point: {point}")


        return path_msg

    # #} end of planPath()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiagnostics():

    def callbackControlManagerDiagnostics(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[motion_planner]: getting ControlManager diagnostics')

        self.sub_control_manager_diag = msg

    # #} end of

    # #{ callbackStart():

    def callbackStart(self, req):

        if not self.is_initialized:
            return Vec1Response(False, "not initialized")

        # set the step size based on the service data
        #step_size = req.goal
        step_size = 0.3

        path_msg = self.planPath(step_size)

        try:
            response = self.sc_path.call(path_msg)
        except:
            rospy.logerr('[motion_planner]: path service not callable')
            pass

        if response.success:
            rospy.loginfo('[motion_planner]: path set')
        else:
            rospy.loginfo('[motion_planner]: path setting failed, message: {}'.format(response.message))

        return Vec1Response(True, "starting")

    # #} end of

    ## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[motion_planner]: main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
            if self.sub_control_manager_diag.tracker_status.have_goal:
                rospy.loginfo('[motion_planner]: tracker has goal')
            else:
                rospy.loginfo('[motion_planner]: waiting for command')

    # #} end of timerMain()

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
