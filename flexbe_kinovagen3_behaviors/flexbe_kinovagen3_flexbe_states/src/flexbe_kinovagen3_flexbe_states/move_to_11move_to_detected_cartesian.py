#!/usr/bin/env python
import rospy
import numpy as np
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler
import actionlib

class MoveToDetectedPositionState_cartesian(EventState):
    '''
    Moves the Gen3 arm to detected object position using trac_ik.
    Takes x, y, z coordinates from previous detection state.
    
    -- roll           float     Target roll orientation (radians)
    -- pitch          float     Target pitch orientation (radians)
    -- yaw            float     Target yaw orientation (radians)
    -- duration       float     Time to reach target (seconds)
    -- z_offset       float     Z offset to add to detected position (e.g., 0.1 for 10cm above)
    
    ># detected_x     float     Detected X coordinate from previous state
    ># detected_y     float     Detected Y coordinate from previous state
    ># detected_z     float     Detected Z coordinate from previous state
    
    <= reached                  Successfully reached target pose
    <= failed                   Failed (IK failed or motion failed)
    '''
    
    def __init__(self, roll=0.0, pitch=np.pi/2, yaw=0.0, duration=5.0, z_offset=0.0):
        super(MoveToDetectedPositionState_cartesian, self).__init__(
            outcomes=['reached', 'failed'],
            input_keys=['detected_x', 'detected_y', 'detected_z']
        )
        
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._duration = duration
        self._z_offset = z_offset
        self._client = None
        self._failed = False
        self._ik_solver = None
        
    def execute(self, userdata):
        if self._failed:
            return 'failed'
            
        if self._client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            Logger.loginfo('Movement to detected position SUCCEEDED')
            return 'reached'
        elif self._client.get_state() == actionlib.GoalStatus.ABORTED:
            Logger.logerr('Movement to detected position ABORTED')
            return 'failed'
            
        return None
        
    def on_enter(self, userdata):
        self._failed = False
        
        # Get coordinates from userdata (from detection state)
        if 'detected_x' not in userdata or 'detected_y' not in userdata or 'detected_z' not in userdata:
            Logger.logerr('Missing detected coordinates in userdata!')
            self._failed = True
            return
            
        x = userdata.detected_x-0.4
        y = userdata.detected_y
        z = userdata.detected_z + self._z_offset  # Add offset if specified
        
        Logger.loginfo(f'Moving to detected position: X={x:.3f}, Y={y:.3f}, Z={z:.3f} (offset={self._z_offset:.3f})')
        
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
        
        # Use a reasonable seed
        q_seed = [0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, -1.5700]
        
        # Solve IK
        Logger.loginfo(f'Solving IK for detected pose...')
        ik_result = self._ik_solver.get_ik(q_seed, x, y, z, qx, qy, qz, qw)
        
        if not ik_result:
            Logger.logerr(f'IK failed for detected pose: ({x:.3f}, {y:.3f}, {z:.3f})')
            self._failed = True
            return
            
        joint_positions = list(ik_result)
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
        Logger.loginfo('Sent joint trajectory goal to detected position')