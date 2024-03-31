#!/usr/bin/python3

import rospy
import numpy as np
import minieigen as eig

#from rrt_star import rrt_star
#from rrt_star import rrt_star_yaw
#from rrt_star import rrt_star_2d
from rrt_star import rrt_star_2d_yaw as rrt_st

from visualization_msgs.msg import Marker

from mrs_msgs.msg import ControlManagerDiagnostics,Reference
from mrs_msgs.msg import DynamicsConstraints
from mrs_msgs.msg import UavState
from mrs_msgs.msg import TrackerCommand

from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.srv import GetPathSrv,GetPathSrvRequest
from mrs_msgs.srv import TrajectoryReferenceSrv,TrajectoryReferenceSrvRequest
from mrs_msgs.srv import Vec1,Vec1Response

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("motion_planner", anonymous=True)

        ## | --------------------- load parameters -------------------- |

        self.frame_id = rospy.get_param("~frame_id")
        self.namespace = "uav1/"

        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = rospy.get_param("~center/z")

        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")

        self.num_nodes = 15
        self.radius = 2
        self.step_size = 1
        self.tolerance = 0.5
        self.pose = np.array([])
        self.best_branch = []

        self.timer_main_rate = rospy.get_param("~timer_main/rate")

        rospy.loginfo('[motion_planner]: initialized')

        ## | ----------------------- publishers ---------------------- |

        self.pub_markers = rospy.Publisher("~visualization_marker_out", Marker, queue_size=50)

        ## | ----------------------- subscribers ---------------------- |

        self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)
        self.sub_uav_state = rospy.Subscriber("~uav_state_in", UavState, self.callbackUavState)
        #self.sub_tracker_cmd = rospy.Subscriber("~tracker_cmd_in", TrackerCommand, self.callbackTrackerCommand)
        #self.sub_constraints = rospy.Subscriber("~constraints_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        ## | --------------------- service servers -------------------- |

        self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)
        #self.ss_path = rospy.Service('~path_in', PathSrv, self.callbackPath)
        #self.ss_get_path = rospy.Service('~get_path_in', GetPathSrv, self.callbackGetPath)

        ## | --------------------- service clients -------------------- |

        #self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)
        self.sc_trajectory_generation = rospy.ServiceProxy('~trajectory_generation_out', GetPathSrv)
        self.sc_trajectory_reference = rospy.ServiceProxy('~trajectory_reference_out', TrajectoryReferenceSrv)

        ## | ------------------------- timers ------------------------- |

        self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rospy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    # #{ recedingHorizon()

    def rh(self, prev_best_branch, goal):

        rospy.loginfo('[motion_planner]: executing path')

        N_max = self.num_nodes
        dim_x = self.dimensions_x
        dim_y = self.dimensions_y
        radius = self.radius
        step_size = self.step_size
        pose = self.pose
        root = rrt_st.Node(pose)
        tree = [root]
        #goal_tree = []

        if len(prev_best_branch) > 0:
            for i in range(2, len(prev_best_branch)): # ranges starts in 2 to ignore previous and current start position
                nearest_node = rrt_st.find_nearest(tree, prev_best_branch[i][:2])
                new_node = rrt_st.steer(nearest_node, prev_best_branch[i][:2], step_size)
                self.visualize_node(new_node.point[:2], 50+i, self.namespace)

                nearby_nodes = rrt_st.find_nearby(tree, new_node, radius)
                new_node = rrt_st.choose_parent(new_node, nearby_nodes)
                tree.append(new_node)
                self.visualize_edge(new_node, 100+i, self.namespace)

                rrt_st.rewire(tree, new_node, nearby_nodes, radius)

        for j in range(N_max):
            # Samples Point
            rand_point = rrt_st.sample_space(dim_x, dim_y)
            # Finds nearest node
            nearest_node = rrt_st.find_nearest(tree, rand_point)
            # Steers nearest node in the direction of the sampled point
            new_node = rrt_st.steer(nearest_node, rand_point, step_size)
            self.visualize_node(new_node.point[:2], j, self.namespace)
            
            #NEEDS TO ADD OBSTACLE AVOIDANCE
            # Finds nearby nodes within a certain radius 
            nearby_nodes = rrt_st.find_nearby(tree, new_node, radius)
            # Chooses the sampled node's parent
            new_node = rrt_st.choose_parent(new_node, nearby_nodes)
            # Appends it to tree and rewires the nearby nodes
            tree.append(new_node)
            self.visualize_edge(new_node, j, self.namespace)
            rrt_st.rewire(tree, new_node, nearby_nodes, radius)
            
            if j == N_max-1:
                # Backtrack with the node having the smallest distance to goal
                distances = [np.linalg.norm(goal[:2] - node.point[:2]) for node in tree]
                min_index = distances.index(min(distances))
                min_cost_node = tree[min_index]
                best_branch, next_best_node = rrt_st.backtrack_path_node(min_cost_node)
                self.visualize_path(min_cost_node, self.namespace)

            j += 1

        # Convert path to a ROS message
        path_msg = GetPathSrvRequest()
        path_msg.path.header.frame_id = "uav1/" + self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        #path_msg.path.stop_at_waypoints = True

        # Follow Reference
        reference = Reference()
        reference.position.x = next_best_node.point[0]
        reference.position.y = next_best_node.point[1]
        reference.position.z = self.center_z
        reference.heading = 0 
        path_msg.path.points.append(reference)

        return path_msg, best_branch

    def visualize_node(self, pos, id, namespace):
        n = Marker()
        n.header.stamp = rospy.Time.now()
        n.header.seq = id
        n.header.frame_id = namespace + self.frame_id
        n.id = id
        n.ns = "nodes"
        n.type = Marker.SPHERE
        n.action = Marker.ADD
        n.pose.position.x = pos[0]
        n.pose.position.y = pos[1]
        n.pose.position.z = self.center_z

        n.pose.orientation.x = 1
        n.pose.orientation.y = 0
        n.pose.orientation.z = 0
        n.pose.orientation.w = 0

        n.scale.x = 0.2
        n.scale.y = 0.2
        n.scale.z = 0.2

        n.color.r = 0.4
        n.color.g = 0.7
        n.color.b = 0.2
        n.color.a = 1

        n.lifetime = rospy.Duration(5.0)
        n.frame_locked = False
        self.pub_markers.publish(n)

    def visualize_edge(self, node, id, namespace):
        e = Marker()
        e.header.stamp = rospy.Time.now()
        e.header.seq = id
        e.header.frame_id = namespace + self.frame_id
        e.id = id
        e.ns = "tree_branches"
        e.type = Marker.ARROW
        e.action = Marker.ADD
        e.pose.position.x = node.parent.point[0]
        e.pose.position.y = node.parent.point[1]
        e.pose.position.z = self.center_z

        q = eig.Quaternion()
        init = np.array([1.0, 0.0, 0.0])
        dir = np.array([node.point[0] - node.parent.point[0],
                        node.point[1] - node.parent.point[1],
                        node.point[2] - node.parent.point[2]])
        q.setFromTwoVectors(init, dir)
        q.normalize()

        e.pose.orientation.x = q[0]
        e.pose.orientation.y = q[1]
        e.pose.orientation.z = q[2]
        e.pose.orientation.w = q[3]

        e.scale.x = np.linalg.norm(dir)
        e.scale.y = 0.05
        e.scale.z = 0.05

        e.color.r = 1.0
        e.color.g = 0.3
        e.color.b = 0.7
        e.color.a = 1.0

        e.lifetime = rospy.Duration(5.0)
        e.frame_locked = False
        self.pub_markers.publish(e)

    def visualize_path(self, node, namespace):
        id = 150
        while node.parent:
            p = Marker()
            p.header.stamp = rospy.Time.now()
            p.header.seq = id
            p.header.frame_id = namespace + self.frame_id
            p.id = id
            p.ns = "path"
            p.type = Marker.ARROW
            p.action = Marker.ADD
            p.pose.position.x = node.parent.point[0]
            p.pose.position.y = node.parent.point[1]
            p.pose.position.z = self.center_z

            q = eig.Quaternion()
            init = np.array([1.0, 0.0, 0.0])
            dir = np.array([node.point[0] - node.parent.point[0],
                            node.point[1] - node.parent.point[1],
                            node.point[2] - node.parent.point[2]])
            q.setFromTwoVectors(init, dir)
            q.normalize()

            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]

            p.scale.x = np.linalg.norm(dir)
            p.scale.y = 0.07
            p.scale.z = 0.07

            p.color.r = 0.7
            p.color.g = 0.7
            p.color.b = 0.3
            p.color.a = 1.0

            p.lifetime = rospy.Duration(100.0)
            p.frame_locked = False
            self.pub_markers.publish(p)
            node = node.parent
            id += 1
        

    # #} end of recedingHorizon()

    # #{ planPath()

    def planPath(self, step_size):

        rospy.loginfo('[motion_planner]: planning path')

        # Define start and goal points

        # 2D with yaw
        start = np.array([self.center_x+20, self.center_y, 0])
        goal = np.array([self.center_x+27, self.center_y+4, 0])

        # Parameter Setup
        max_iterations = 1000
        #step_size = 0.3
        radius = 0.5
        dim_x = self.dimensions_x
        dim_y = self.dimensions_y
        #dim_z = self.dimensions_y

        # Define obstacles
        obstacles = [] 

        # Call RRT* algorithm to generate path
        #tree, path = rrt_star.rrt_star(start, goal, obstacles, dim_x, dim_y, dim_z, max_iter=max_iterations, step_size=step_size, radius=radius)
        tree, path = rrt_st.rrt_star(start, goal, obstacles,  dim_x, dim_y, max_iter=max_iterations, step_size=step_size, radius=radius)
        
        # Convert path to a ROS message
        path_msg = GetPathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        #path_msg.path.stop_at_waypoints = True

        # Follow Reference
        for point in path:
            reference = Reference()
            reference.position.x = point[0]
            reference.position.y = point[1]
            #reference.position.z = point[3] # For 3D with yaw
            reference.position.z = self.center_z # For 2D
            reference.heading = point[2]  # Adjust heading if needed
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

    # #{ callbackUavState():

    def callbackUavState(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[motion_planner]: getting UavState diagnostics')

        self.sub_uav_state = msg
        self.pose = np.array([self.sub_uav_state.pose.position.x, self.sub_uav_state.pose.position.y, 0])

    # #} end of

    # #{ callbackStart():

    def callbackStart(self, req):

        if not self.is_initialized:
            return Vec1Response(False, "not initialized")

        # set the step size based on the service data
        #step_size = req.goal
        #step_size = 0.4
        goal = np.array([25, 27, 0])

        #path_msg = self.planPath(step_size)
        while np.linalg.norm(goal[:2] - self.pose[:2]) > self.tolerance:
            if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
                if self.sub_control_manager_diag.tracker_status.have_goal:
                    # Waits until planned trajectory is complete
                    rospy.sleep(2)
                else:
                    path_msg, self.best_branch = self.rh(self.best_branch, goal)

                    #Transform path into trajectory
                    try:
                        response_plan = self.sc_trajectory_generation.call(path_msg)
                        trajectory = response_plan.trajectory
                    except:
                        rospy.logerr('[motion_planner]: path service not callable')
                        pass

                    #Take trajectory and give it to the trackers
                    try:
                        response_trajectory = self.sc_trajectory_reference.call(trajectory)
                    except:
                        rospy.logerr('[motion_planner]: trajectory service not callable')
                        pass

                    #Check if the trajectory reference is available for the UAV to fly
                    if response_trajectory.success:
                        rospy.loginfo('[motion_planner]: trajectory set')
                    else:
                        rospy.loginfo('[motion_planner]: trajectory setting failed, message: {}'.format(response.message))
                    
                    # Wait an iteration of the callback so the function is not called more than once
                    rospy.sleep(self.timer_main_rate)

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
