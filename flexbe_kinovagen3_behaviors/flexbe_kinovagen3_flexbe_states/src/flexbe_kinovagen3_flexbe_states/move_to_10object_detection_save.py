#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from flexbe_core import EventState, Logger
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs

class DetectObjectSimpleState_save(EventState):
    '''
    Detects objects using HSV color detection and reports their 3D coordinates in base_link frame.
    Saves the first detected object's position for use in next state.
    
    -- base_frame      string    Target frame (default: "base_link")
    -- min_area        int       Minimum blob area (default: 800)
    -- timeout         float     Time to wait for detection (default: 5.0)
    
    #> detected_x      float     Detected object X coordinate in base frame
    #> detected_y      float     Detected object Y coordinate in base frame
    #> detected_z      float     Detected object Z coordinate in base frame
    
    <= detected                  Object detected, coordinates saved
    <= failed                    Detection failed or timeout
    '''
    
    def __init__(self, base_frame="base_link", min_area=800, timeout=5.0):
        super(DetectObjectSimpleState_save, self).__init__(
            outcomes=['detected', 'failed'],
            output_keys=['detected_x', 'detected_y', 'detected_z']
        )
        
        self._base_frame = base_frame
        self._min_area = min_area
        self._timeout = timeout
        self._hsv_ranges = [[17, 0, 0, 49, 255, 255]]  # Default HSV range
        
        self._bridge = CvBridge()
        self._tfbuf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tfbuf)
        
        self._rgb_msg = None
        self._cloud_msg = None
        self._detected = False
        self._start_time = None
        self._detected_coords = None
        
    def _rgb_callback(self, msg):
        self._rgb_msg = msg
        
    def _cloud_callback(self, msg):
        self._cloud_msg = msg
        
    def _mask_from_ranges(self, hsv):
        """Create mask from HSV ranges"""
        m = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hL, sL, vL, hH, sH, vH in self._hsv_ranges:
            m = cv2.bitwise_or(m, cv2.inRange(hsv, (hL, sL, vL), (hH, sH, vH)))
        m = cv2.medianBlur(m, 5)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        return m
        
    def _xyz_from_cloud_uv(self, cloud, u, v):
        """Get (x,y,z) at pixel (u,v) from organized PointCloud2"""
        if u < 0 or v < 0 or u >= cloud.width or v >= cloud.height:
            return (np.nan, np.nan, np.nan)
            
        for p in pc2.read_points(cloud, field_names=("x", "y", "z"), 
                                 skip_nans=False, uvs=[(int(u), int(v))]):
            return (float(p[0]), float(p[1]), float(p[2]))
        return (np.nan, np.nan, np.nan)
        
    def _detect_and_report(self):
        """Process RGB and cloud to detect objects and report coordinates"""
        if self._rgb_msg is None or self._cloud_msg is None:
            return False
            
        # Convert RGB
        img = self._bridge.imgmsg_to_cv2(self._rgb_msg, "bgr8")
        H, W = img.shape[:2]
        
        # Check cloud size matches
        if self._cloud_msg.width != W or self._cloud_msg.height != H:
            Logger.logwarn("Cloud and RGB size mismatch")
            return False
            
        # HSV mask
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = self._mask_from_ranges(hsv)
        
        # Find contours
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) >= self._min_area]
        
        if len(cnts) == 0:
            Logger.logwarn("No objects detected")
            return False
            
        # Sort left to right
        cnts.sort(key=lambda c: cv2.boundingRect(c)[0])
        
        Logger.loginfo(f"===== Detected {len(cnts)} object(s) =====")
        
        # Process first object and save coordinates
        first_saved = False
        
        for idx, c in enumerate(cnts):
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w // 2, y + h // 2
            
            # Get 3D coordinates from cloud
            Xc, Yc, Zc = self._xyz_from_cloud_uv(self._cloud_msg, u, v)
            
            if not np.isfinite(Zc):
                Logger.logwarn(f"Object {idx}: Invalid depth")
                continue
                
            # Create point in camera frame
            pt_cam = PointStamped()
            pt_cam.header = self._cloud_msg.header
            pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc
            
            # Transform to base frame
            try:
                pt_base = self._tfbuf.transform(pt_cam, self._base_frame, rospy.Duration(0.5))
                bx, by, bz = pt_base.point.x, pt_base.point.y, pt_base.point.z
                
                # Save first valid object coordinates
                if not first_saved:
                    self._detected_coords = (bx, by, bz)
                    first_saved = True
                    Logger.loginfo(f"**SAVED** Object {idx} coordinates for next state")
                
                Logger.loginfo(f"Object {idx}:")
                Logger.loginfo(f"  Camera frame: X={Xc:.3f}, Y={Yc:.3f}, Z={Zc:.3f}")
                Logger.loginfo(f"  Base frame:   X={bx:.3f}, Y={by:.3f}, Z={bz:.3f}")
                
            except Exception as e:
                Logger.logerr(f"Object {idx}: TF transform failed: {e}")
                
        Logger.loginfo("=====================================")
        return first_saved  # Return True only if we saved coordinates
            
    def execute(self, userdata):
        if self._detected:
            # Pass coordinates to next state via userdata
            if self._detected_coords:
                userdata.detected_x = self._detected_coords[0]
                userdata.detected_y = self._detected_coords[1]
                userdata.detected_z = self._detected_coords[2]
                Logger.loginfo(f"Passing coordinates to next state: X={userdata.detected_x:.3f}, Y={userdata.detected_y:.3f}, Z={userdata.detected_z:.3f}")
            return 'detected'
            
        # Check timeout
        if self._start_time is not None:
            elapsed = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed > self._timeout:
                Logger.logerr(f"Detection timeout after {self._timeout}s")
                return 'failed'
                
        # Try detection
        if self._detect_and_report():
            self._detected = True
            return None  # Return None first, then 'detected' on next execute
            
        return None
        
    def on_enter(self, userdata):
        self._detected = False
        self._start_time = rospy.Time.now()
        self._rgb_msg = None
        self._cloud_msg = None
        self._detected_coords = None
        
        Logger.loginfo("Starting object detection...")
        
        # Subscribe to topics
        self._rgb_sub = rospy.Subscriber(
            "/my_gen3/camera_link/rgb/image_raw",
            Image,
            self._rgb_callback
        )
        
        self._cloud_sub = rospy.Subscriber(
            "/my_gen3/camera_link/rgb/points",
            PointCloud2,
            self._cloud_callback
        )
        
    def on_exit(self, userdata):
        # Unsubscribe
        if hasattr(self, '_rgb_sub'):
            self._rgb_sub.unregister()
        if hasattr(self, '_cloud_sub'):
            self._cloud_sub.unregister()