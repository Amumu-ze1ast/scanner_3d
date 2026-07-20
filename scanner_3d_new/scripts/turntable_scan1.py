#!/usr/bin/env python
import rospy
import subprocess
from flexbe_core import EventState, Logger

class Turntalbe_LaunchRTABMapState(EventState):
    '''
    State to launch RTAB-Map for turntable scanning using subprocess.
    
    <= succeeded         RTAB-Map launched successfully
    <= failed            Failed to launch RTAB-Map
    
    '''
    
    def __init__(self):
        super(Turntalbe_LaunchRTABMapState, self).__init__(outcomes=['succeeded', 'failed'])
        
        self._process = None
        
    def execute(self, userdata):
        # Check if process is running
        if self._process is not None:
            poll_result = self._process.poll()
            if poll_result is None:
                # Process is still running
                Logger.loginfo('RTAB-Map is running')
                return 'succeeded'
            else:
                Logger.logerr('RTAB-Map process exited with code: %d' % poll_result)
                return 'failed'
        else:
            Logger.logerr('RTAB-Map process not started')
            return 'failed'
    
    def on_enter(self, userdata):
        Logger.loginfo('Launching RTAB-Map for turntable scanning...')
        
        # Build roslaunch command
        cmd = [
            'roslaunch', 'rtabmap_ros', 'rtabmap.launch',
            'rtabmap_args:=--delete_db_on_start',
            'depth_topic:=/robot2_turntable/rgbd_camera1/depth/image_raw',
            'rgb_topic:=/robot2_turntable/rgbd_camera1/rgb/image_raw',
            'camera_info_topic:=/robot2_turntable/rgbd_camera1/rgb/camera_info',
            'approx_sync:=true',
            'visual_odometry:=true',
            'queue_size:=20',
            'cloud_voxel_size:=0.01',
            'frame_id:=base_link',
            'wait_for_transform:=0.2'
        ]
        
        try:
            # Launch process
            self._process = subprocess.Popen(cmd)
            
            # Wait for RTAB-Map to initialize
            rospy.sleep(10.0)
            
            Logger.loginfo('RTAB-Map launch initiated')
            
        except Exception as e:
            Logger.logerr('Failed to launch RTAB-Map: %s' % str(e))
            self._process = None
    
    def on_exit(self, userdata):
        # Terminate RTAB-Map when exiting state
        if self._process is not None:
            try:
                self._process.terminate()
                self._process.wait(timeout=5.0)
                Logger.loginfo('RTAB-Map terminated')
            except Exception as e:
                Logger.logwarn('Error terminating RTAB-Map: %s' % str(e))
                try:
                    self._process.kill()
                except:
                    pass