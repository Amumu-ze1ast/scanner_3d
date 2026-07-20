#!/usr/bin/env python3
import rospy
import numpy as np
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler
import actionlib

class MoveInStraightLineState(EventState):
    '''
    Moves the Gen3 arm end-effector in a straight line from start to end position.
    Generates intermediate waypoints and uses trac_ik to solve for each point.
    
    -- start_x        float     Start X position (meters)
    -- start_y        float     Start Y position (meters)
    -- start_z        float     Start Z position (meters)
    -- end_x          float     End X position (meters)
    -- end_y          float     End Y position (meters)
    -- end_z          float     End Z position (meters)
    -- num_waypoints  int       Number of intermediate waypoints (default: 5)
    -- roll           float     Orientation roll (radians)
    -- pitch          float     Orientation pitch (radians)
    -- yaw            float     Orientation yaw (radians)
    -- duration       float     Total duration for entire motion (seconds)
    
    <= reached                  Successfully completed linear motion
    <= failed                   Failed (IK failed for a waypoint or motion failed)
    '''
    
    def __init__(self, start_x, start_y, start_z, end_x, end_y, end_z, 
                 num_waypoints=5, roll=0.0, pitch=np.pi/2, yaw=0.0, duration=10.0):
        super(MoveInStraightLineState, self).__init__(outcomes=['reached', 'failed'])
        
        self._start = np.array([start_x, start_y, start_z])
        self._end = np.array([end_x, end_y, end_z])
        self._num_waypoints = num_waypoints
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._duration = duration
        self._client = None
        self._failed = False
        self._ik_solver = None
        
    def generate_waypoints(self, start, end, num_between):
        """Generate waypoints between start and end points."""
        total_points = num_between + 2  # Include start and end
        points = np.linspace(start, end, total_points)
        points = np.round(points, 3)
        return [tuple(point) for point in points]
        
    def execute(self, userdata):
        if self._failed:
            return 'failed'
            
        if self._client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'reached'
        elif self._client.get_state() == actionlib.GoalStatus.ABORTED:
            return 'failed'
            
        return None
        
    def on_enter(self, userdata):
        self._failed = False
        
        # Initialize trac_ik solver
        try:
            urdf_string = rospy.get_param("my_gen3/robot_description")
            self._ik_solver = IK("base_link", "end_effector_link", urdf_string=urdf_string)
        except Exception as e:
            Logger.logerr(f'Failed to initialize trac_ik: {e}')
            self._failed = True
            return
            
        # Generate waypoints
        waypoints = self.generate_waypoints(self._start, self._end, self._num_waypoints)
        Logger.loginfo(f'Generated {len(waypoints)} waypoints for straight line motion')
        
        # Convert orientation to quaternion
        qx, qy, qz, qw = quaternion_from_euler(self._roll, self._pitch, self._yaw)
        
        # Initial seed
        q_seed = [0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, -1.5700]
        
        # Solve IK for all waypoints
        joint_trajectory = []
        for i, waypoint in enumerate(waypoints):
            x, y, z = waypoint
            
            ik_result = self._ik_solver.get_ik(q_seed, x, y, z, qx, qy, qz, qw)
            
            if not ik_result:
                Logger.logerr(f'IK failed for waypoint {i+1}: ({x}, {y}, {z})')
                self._failed = True
                return
                
            joint_positions = list(ik_result)
            joint_trajectory.append(joint_positions)
            
            # Use this solution as seed for next waypoint
            q_seed = ik_result
            
            Logger.loginfo(f'Waypoint {i+1}/{len(waypoints)}: ({x}, {y}, {z}) -> IK: {[round(j, 3) for j in joint_positions]}')
        
        # Create action client
        self._client = actionlib.SimpleActionClient(
            '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            Logger.logwarn('Joint trajectory action server not available!')
            self._failed = True
            return
            
        # Create joint trajectory goal with all waypoints
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7'
        ]
        
        # Add all waypoint positions to trajectory
        time_per_segment = self._duration / (len(waypoints) - 1)
        for i, joint_positions in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = rospy.Duration(time_per_segment * i)
            goal.trajectory.points.append(point)
        
        # Send goal
        self._client.send_goal(goal)
        Logger.loginfo(f'Sent linear trajectory with {len(waypoints)} waypoints')