#!/usr/bin/env python3
import rospy
import numpy as np
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler
import actionlib

class MoveToCartesianPoseState(EventState):
    '''
    Moves the Gen3 arm end-effector to a specified Cartesian pose using trac_ik.
    
    -- x              float     Target X position (meters)
    -- y              float     Target Y position (meters)
    -- z              float     Target Z position (meters)
    -- roll           float     Target roll orientation (radians)
    -- pitch          float     Target pitch orientation (radians)
    -- yaw            float     Target yaw orientation (radians)
    -- duration       float     Time to reach target (seconds)
    
    <= reached                  Successfully reached target pose
    <= failed                   Failed to reach target pose (IK failed or motion failed)
    '''
    
    def __init__(self, x, y, z, roll=0.0, pitch=np.pi/2, yaw=0.0, duration=5.0):
        super(MoveToCartesianPoseState, self).__init__(outcomes=['reached', 'failed'])
        
        self._x = x
        self._y = y
        self._z = z
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._duration = duration
        self._client = None
        self._failed = False
        self._ik_solver = None
        
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
            
        # Convert orientation to quaternion
        qx, qy, qz, qw = quaternion_from_euler(self._roll, self._pitch, self._yaw)
        
        # Use a reasonable seed (you can modify this)
        q_seed = [0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, -1.5700]
        
        # Solve IK
        Logger.loginfo(f'Solving IK for pose: x={self._x}, y={self._y}, z={self._z}')
        ik_result = self._ik_solver.get_ik(q_seed, self._x, self._y, self._z, qx, qy, qz, qw)

        ik_result_modified = list (ik_result)
        ik_result_modified[6]=ik_result[6]-1.57
        
        if not ik_result:
            Logger.logerr(f'IK failed for target pose: ({self._x}, {self._y}, {self._z})')
            self._failed = True
            return
            
        joint_positions = list(ik_result_modified)
        Logger.loginfo(f'IK solution: {[round(j, 4) for j in joint_positions]}')
        
        # Create action client for joint trajectory
        self._client = actionlib.SimpleActionClient(
            '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            Logger.logwarn('Joint trajectory action server not available!')
            self._failed = True
            return
            
        # Create joint trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(self._duration)
        goal.trajectory.points = [point]
        
        # Send goal
        self._client.send_goal(goal)
        Logger.loginfo('Sent joint trajectory goal from IK solution')