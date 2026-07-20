#!/usr/bin/env python
import rospy
import numpy as np
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler
import actionlib
import tf

class MoveToDetectedPositionState_straight(EventState):
    '''
    Moves the Gen3 arm end-effector in a straight line from current position to detected object.
    Uses current joint angles as initial seed and each waypoint's solution as seed for the next.
    
    -- num_waypoints  int       Number of intermediate waypoints (default: 5)
    -- roll           float     Orientation roll (radians)
    -- pitch          float     Orientation pitch (radians)
    -- yaw            float     Orientation yaw (radians)
    -- duration       float     Total duration for entire motion (seconds)
    -- x_offset       float     X offset to add to detected position (e.g., 0.1 for 10cm above)
    
    ># detected_x     float     Detected X coordinate from detection state
    ># detected_y     float     Detected Y coordinate from detection state
    ># detected_z     float     Detected Z coordinate from detection state
    
    <= reached                  Successfully completed linear motion
    <= failed                   Failed (IK failed for a waypoint or motion failed)
    '''
    
    def __init__(self, num_waypoints=5, roll=0.0, pitch=np.pi/2, yaw=0.0, duration=10.0, x_offset=0.0):
        super(MoveToDetectedPositionState_straight, self).__init__(
            outcomes=['reached', 'failed'],
            input_keys=['detected_x', 'detected_y', 'detected_z']
        )
        
        self._num_waypoints = num_waypoints
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._duration = duration
        self._x_offset = x_offset
        self._client = None
        self._failed = False
        self._ik_solver = None
        self._tf_listener = None
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
            Logger.loginfo('Linear movement to detected object SUCCEEDED')
            return 'reached'
        elif self._client.get_state() == actionlib.GoalStatus.ABORTED:
            Logger.logerr('Linear movement to detected object ABORTED')
            return 'failed'
            
        return None
        
    def on_enter(self, userdata):
        self._failed = False
        
        # Get detected object coordinates from userdata
        if 'detected_x' not in userdata or 'detected_y' not in userdata or 'detected_z' not in userdata:
            Logger.logerr('Missing detected coordinates in userdata!')
            self._failed = True
            return
        
        # Set target position from detected coordinates + offset
        target_x = userdata.detected_x-self._x_offset
        target_y = userdata.detected_y
        target_z = userdata.detected_z
        self._end = np.array([target_x, target_y, target_z])
        
        Logger.loginfo(f'Target (detected object): X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}')
        Logger.loginfo(f'X offset applied: {self._x_offset:.3f}m')
        
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
        Logger.loginfo(f'Current EE position: X={self._start[0]:.3f}, Y={self._start[1]:.3f}, Z={self._start[2]:.3f}')
        
        # Get current joint positions to use as seed
        q_seed = self.get_current_joint_positions()
        if q_seed is None:
            Logger.logerr('Failed to get current joint positions!')
            self._failed = True
            return
            
        Logger.loginfo(f'Using current joint positions as seed')
        
        # Initialize trac_ik solver
        try:
            urdf_string = rospy.get_param("my_gen3/robot_description")
            self._ik_solver = IK("base_link", "end_effector_link", urdf_string=urdf_string)
        except Exception as e:
            Logger.logerr(f'Failed to initialize trac_ik: {e}')
            self._failed = True
            return
            
        # Generate waypoints from current position to detected target
        waypoints = self.generate_waypoints(self._start, self._end, self._num_waypoints)
        Logger.loginfo(f'Generated {len(waypoints)} waypoints for straight line motion to detected object')
        
        # Convert orientation to quaternion
        qx, qy, qz, qw = quaternion_from_euler(self._roll, self._pitch, self._yaw)
        
        # Solve IK for all waypoints, using each solution as seed for next
        joint_trajectory = []
        for i, waypoint in enumerate(waypoints):
            x, y, z = waypoint
            
            ik_result = self._ik_solver.get_ik(q_seed, x, y, z, qx, qy, qz, qw)
            
            if not ik_result:
                Logger.logerr(f'IK failed for waypoint {i+1}: ({x}, {y}, {z})')
                self._failed = True
                return
            
            # Apply joint 7 modification (gripper orientation adjustment)
            ik_result_modified = list(ik_result)
            ik_result_modified[6] = ik_result[6] - 1.57  # Adjust last joint
                
            joint_positions = ik_result_modified
            joint_trajectory.append(joint_positions)
            
            # Use this solution as seed for next waypoint
            q_seed = ik_result
            
            Logger.loginfo(f'Waypoint {i+1}/{len(waypoints)}: ({x:.3f}, {y:.3f}, {z:.3f})')
        
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
        Logger.loginfo(f'Sent linear trajectory with {len(waypoints)} waypoints to detected object')
        
        # Unsubscribe from joint states
        joint_sub.unregister()