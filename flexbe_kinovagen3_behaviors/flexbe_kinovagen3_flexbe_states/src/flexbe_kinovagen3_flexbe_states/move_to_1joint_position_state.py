#!/usr/bin/env python3
import rospy
from flexbe_core import EventState, Logger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

class MoveToJointPositionState(EventState):
    '''
    Moves the Gen3 arm to specified joint positions.
    
    -- joint_positions   float[]   Target joint positions in radians [j1, j2, j3, j4, j5, j6, j7]
    -- duration          float     Time to reach target (seconds)
    
    <= reached                     Successfully reached target position
    <= failed                      Failed to reach target position
    '''
    
    def __init__(self, joint_positions, duration=5.0):
        super(MoveToJointPositionState, self).__init__(outcomes=['reached', 'failed'])
        
        self._joint_positions = joint_positions
        self._duration = duration
        self._client = None
        self._failed = False
        
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
        
        # Create action client
        self._client = actionlib.SimpleActionClient(
            '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            Logger.logwarn('Action server not available!')
            self._failed = True
            return
            
        # Create goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = self._joint_positions
        point.time_from_start = rospy.Duration(self._duration)
        goal.trajectory.points = [point]
        
        # Send goal
        self._client.send_goal(goal)
        Logger.loginfo('Sent joint position goal')