#!/usr/bin/env python
import rospy
import actionlib
from flexbe_core import EventState, Logger
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class GripperControlState(EventState):
    '''
    Controls the Robotiq 2F-85 gripper to open or close.
    
    -- position       float     Gripper position (0.0 = fully open, 0.8 = fully closed)
    -- effort         float     Maximum effort to apply (default: 10.0)
    -- wait_time      float     Time to wait after sending command (default: 2.0 seconds)
    
    <= done                     Gripper command completed
    <= failed                   Gripper command failed
    '''
    
    def __init__(self, position, effort=10.0, wait_time=2.0):
        super(GripperControlState, self).__init__(outcomes=['done', 'failed'])
        
        self._position = position
        self._effort = effort
        self._wait_time = wait_time
        self._client = None
        self._failed = False
        self._start_time = None
        
    def execute(self, userdata):
        if self._failed:
            Logger.logerr('Gripper failed flag set')
            return 'failed'
        
        # Simple time-based waiting
        if self._start_time is not None:
            elapsed = (rospy.Time.now() - self._start_time).to_sec()
            
            # Log action state for debugging
            if self._client is not None:
                state = self._client.get_state()
                # Logger.loginfo(f'Gripper state: {state}, elapsed: {elapsed:.1f}s')
            
            # Exit after wait_time regardless of action status
            if elapsed >= self._wait_time:
                Logger.loginfo(f'Gripper wait time completed ({self._wait_time}s)')
                return 'done'
        
        return None
        
    def on_enter(self, userdata):
        self._failed = False
        self._start_time = rospy.Time.now()
        
        # Create action client
        topic_name = '/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd'
        self._client = actionlib.SimpleActionClient(topic_name, GripperCommandAction)
        
        Logger.loginfo("Waiting for gripper action server...")
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            Logger.logerr("Gripper server not found! Is the driver running?")
            self._failed = True
            return
        
        # Create goal message
        goal = GripperCommandGoal()
        goal.command.position = self._position
        goal.command.max_effort = self._effort
        
        Logger.loginfo(f"Sending gripper command: position={self._position}, effort={self._effort}, will wait {self._wait_time}s")
        self._client.send_goal(goal)