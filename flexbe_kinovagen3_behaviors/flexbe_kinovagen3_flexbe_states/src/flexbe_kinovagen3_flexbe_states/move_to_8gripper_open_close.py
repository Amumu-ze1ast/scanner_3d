#!/usr/bin/env python
import rospy
import actionlib
from flexbe_core import EventState, Logger
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class GripperControlState_open_close(EventState):
    '''
    Controls the Robotiq 2F-85 gripper to open and close multiple times.
    
    -- open_position   float     Gripper open position (default: 0.0)
    -- close_position  float     Gripper close position (default: 0.8)
    -- effort          float     Maximum effort to apply (default: 10.0)
    -- wait_time       float     Time to wait for each movement (default: 2.0 seconds)
    -- repetitions     int       Number of open-close cycles (default: 3)
    
    <= done                      Gripper completed all cycles
    <= failed                    Gripper command failed
    '''
    
    def __init__(self, open_position=0.0, close_position=0.7, effort=10.0, wait_time=2.0, repetitions=3):
        super(GripperControlState_open_close, self).__init__(outcomes=['done', 'failed'])
        
        self._open_position = open_position
        self._close_position = close_position
        self._effort = effort
        self._wait_time = wait_time
        self._repetitions = repetitions
        self._client = None
        self._failed = False
        self._start_time = None
        self._current_cycle = 0
        self._is_closing = True  # Start by closing
        
    def execute(self, userdata):
        if self._failed:
            Logger.logerr('Gripper failed flag set')
            return 'failed'
        
        # Check if wait time has elapsed
        if self._start_time is not None:
            elapsed = (rospy.Time.now() - self._start_time).to_sec()
            
            # Wait time completed, move to next action
            if elapsed >= self._wait_time:
                if self._is_closing:
                    # Just finished closing, now open
                    self._is_closing = False
                    self.send_gripper_command(self._open_position)
                    Logger.loginfo(f'Cycle {self._current_cycle + 1}/{self._repetitions}: Opening gripper')
                else:
                    # Just finished opening, increment cycle
                    self._current_cycle += 1
                    
                    # Check if all cycles completed
                    if self._current_cycle >= self._repetitions:
                        Logger.loginfo(f'Gripper completed all {self._repetitions} cycles')
                        return 'done'
                    
                    # Start next cycle by closing
                    self._is_closing = True
                    self.send_gripper_command(self._close_position)
                    Logger.loginfo(f'Cycle {self._current_cycle + 1}/{self._repetitions}: Closing gripper')
                
                # Reset timer for next movement
                self._start_time = rospy.Time.now()
        
        return None
    
    def send_gripper_command(self, position):
        """Send a gripper command"""
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = self._effort
        self._client.send_goal(goal)
        
    def on_enter(self, userdata):
        self._failed = False
        self._start_time = rospy.Time.now()
        self._current_cycle = 0
        self._is_closing = True
        
        # Create action client
        topic_name = '/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd'
        self._client = actionlib.SimpleActionClient(topic_name, GripperCommandAction)
        
        Logger.loginfo("Waiting for gripper action server...")
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            Logger.logerr("Gripper server not found! Is the driver running?")
            self._failed = True
            return
        
        Logger.loginfo(f"Starting {self._repetitions} open-close cycles")
        Logger.loginfo(f'Cycle 1/{self._repetitions}: Closing gripper')
        
        # Send first command (close)
        self.send_gripper_command(self._close_position)