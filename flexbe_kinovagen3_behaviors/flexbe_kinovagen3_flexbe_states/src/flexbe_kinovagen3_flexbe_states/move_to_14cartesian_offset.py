#!/usr/bin/env python3
import rospy
import numpy as np
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler
import actionlib
from sensor_msgs.msg import JointState
import tf

class MoveToCartesianPoseState_offset(EventState):
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
    
    def __init__(self, x_offset, y_offset, z_offset, roll=0.0, pitch=np.pi/2, yaw=0.0, duration=5.0):
        super(MoveToCartesianPoseState_offset, self).__init__(outcomes=['reached', 'failed'])
        
        self._x_offset = x_offset
        self._y_offset = y_offset
        self._z_offset = z_offset
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._duration = duration
        self._client = None
        self._failed = False
        self._ik_solver = None
        self._current_joint_state = None

    def joint_state_callback(self, msg):
        """Callback to store current joint states"""
        self._current_joint_state = msg
        
    def get_current_ee_position(self):
        """Get current end-effector position using TF"""
        try:
            self._tf_listener.waitForTransform('base_link', 'end_effector_link', 
                                              rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self._tf_listener.lookupTransform('base_link', 'end_effector_link', 
                                                            rospy.Time(0))
            return np.array(trans)
        except Exception as e:
            Logger.logerr(f'Failed to get current EE position: {e}')
            return None
            
    def get_current_joint_positions(self):
        """Get current joint positions in correct order"""
        if self._current_joint_state is None:
            return None
            
        joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7'
        ]
        
        positions = []
        for joint_name in joint_names:
            try:
                idx = self._current_joint_state.name.index(joint_name)
                positions.append(self._current_joint_state.position[idx])
            except ValueError:
                Logger.logerr(f'Joint {joint_name} not found in joint states')
                return None
        
        return positions
        
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
        
        # Initialize TF listener
        self._tf_listener = tf.TransformListener()
        
        # Subscribe to joint states
        self._current_joint_state = None
        joint_sub = rospy.Subscriber('/my_gen3/joint_states', JointState, self.joint_state_callback)
        
        # Wait for joint state
        rospy.sleep(0.5)
        if self._current_joint_state is None:
            Logger.logerr('No joint state received!')
            self._failed = True
            return

        # Get current end-effector position
        current_ee_pos = self.get_current_ee_position()
        if current_ee_pos is None:
            Logger.logerr('Failed to get current EE position!')
            self._failed = True
            return
            
        self._start = current_ee_pos
        Logger.loginfo(f'Current EE position: {self._start}')
        
        # Get current joint positions to use as seed
        q_seed = self.get_current_joint_positions()
        # q_seed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if q_seed is None:
            Logger.logerr('Failed to get current joint positions!')
            self._failed = True
            return
            
        Logger.loginfo(f'Using current joint positions as seed: {[round(j, 3) for j in q_seed]}')
        
        # Initialize trac_ik solver
        try:
            urdf_string = rospy.get_param("my_gen3/robot_description")
            self._ik_solver = IK("base_link", "end_effector_link", urdf_string=urdf_string)
        except Exception as e:
            Logger.logerr(f'Failed to initialize trac_ik: {e}')
            self._failed = True
            return

        end_x = current_ee_pos[0] + self._x_offset
        end_y = current_ee_pos[1] + self._y_offset
        end_z = current_ee_pos[2] + self._z_offset
        
        # self._end = np.array([end_x, end_y, end_z])
        
    
        # Convert orientation to quaternion
        qx, qy, qz, qw = quaternion_from_euler(self._roll, self._pitch, self._yaw)
        
        # Solve IK for all waypoints, using each solution as seed for next
            
        # x, y, z = self._end
        
        ik_result = self._ik_solver.get_ik(q_seed, end_x, end_y, end_z, qx, qy, qz, qw)
        ik_result_modified = list(ik_result)
        ik_result_modified[6] = ik_result[6]-1.57
        
        if not ik_result:
            Logger.logerr(f'IK failed for waypoint {i+1}: ({end_x}, {end_y}, {end_z})')
            self._failed = True
            return
            
        joint_positions = list(ik_result_modified)
        
        # Use this solution as seed for next waypoint
        q_seed = ik_result
        
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
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(self._duration)
        goal.trajectory.points = [point]

        # Send goal
        self._client.send_goal(goal)
        
        # Unsubscribe from joint states
        joint_sub.unregister()