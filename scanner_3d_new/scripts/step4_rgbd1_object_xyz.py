#!/usr/bin/env python3
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from image_geometry import PinholeCameraModel

class HSVWithDepthXYZ:
    """
    RGB-D HSV detector with 3D XYZ (camera optical frame).
    Composite debug image: [RGB+boxes+XYZ | depth colormap | mask]
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_cam_info = False

        # Topics
        self.rgb_topic    = rospy.get_param("~rgb_topic",    "robot2_turntable/rgbd_camera1/rgb/image_raw")
        self.depth_topic  = rospy.get_param("~depth_topic",  "robot2_turntable/rgbd_camera1/depth/image_raw")
        # Use the camera_info that matches the image you’re using for pixel indices.
        self.caminfo_topic = rospy.get_param("~camera_info", "robot2_turntable/rgbd_camera1/rgb/camera_info")

        # HSV ranges: list of [Hlo,Slo,Vlo,Hhi,Shi,Vhi]
        # default_ranges = [[0, 120, 70, 10, 255, 255]]
        default_ranges = [[20, 40, 0, 30, 120, 190]]
        self.ranges   = rospy.get_param("~ranges", default_ranges)
        self.min_area = int(rospy.get_param("~min_area", 800))
        self.depth_win = int(rospy.get_param("~depth_window", 5))  # odd window around center

        # Subscribers
        sub_rgb   = message_filters.Subscriber(self.rgb_topic, Image)
        sub_depth = message_filters.Subscriber(self.depth_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], queue_size=20, slop=0.2)
        sync.registerCallback(self.cb_rgbd)

        rospy.Subscriber(self.caminfo_topic, CameraInfo, self.cb_cam_info, queue_size=1)

        self.pub_dbg = rospy.Publisher("~debug_image", Image, queue_size=1)

        rospy.loginfo("HSV+Depth+XYZ:\n  RGB:   %s\n  Depth: %s\n  Info:  %s\n  ranges: %s",
                      self.rgb_topic, self.depth_topic, self.caminfo_topic, str(self.ranges))

    def cb_cam_info(self, msg):
        if not self.have_cam_info:
            self.cam_model.fromCameraInfo(msg)
            self.have_cam_info = True
            rospy.loginfo("Camera intrinsics loaded: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                          self.cam_model.fx(), self.cam_model.fy(),
                          self.cam_model.cx(), self.cam_model.cy())

    def _depth_to_meters(self, depth_msg):
        try:
            if depth_msg.encoding == "32FC1":
                d = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1").astype(np.float32)
            elif depth_msg.encoding == "16UC1":
                d16 = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
                d = d16.astype(np.float32) / 1000.0
            else:
                d = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding).astype(np.float32)
                if np.nanmax(d) > 20.0:
                    d = d / 1000.0
            d[d <= 0.0] = np.nan
            return d
        except CvBridgeError:
            return None

    def cb_rgbd(self, rgb_msg, depth_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError:
            return
        depth = self._depth_to_meters(depth_msg)
        if depth is None:
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hL,sL,vL,hH,sH,vH in self.ranges:
            mask = cv2.inRange(hsv, (hL,sL,vL), (hH,sH,vH))
            mask_total = cv2.bitwise_or(mask_total, mask)

        mask_total = cv2.medianBlur(mask_total, 5)
        mask_total = cv2.morphologyEx(
            mask_total, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        )

        vis = img.copy()
        cnts, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        win = self.depth_win if self.depth_win % 2 == 1 else self.depth_win + 1
        r = win // 2
        H, W = mask_total.shape[:2]

        for i, c in enumerate(cnts, 1):
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x,y,w,h = cv2.boundingRect(c)
            cx_pix, cy_pix = x + w//2, y + h//2

            cv2.rectangle(vis, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.circle(vis, (cx_pix,cy_pix), 4, (0,0,255), -1)

            # Median depth around the center
            x0 = max(0, cx_pix - r); x1 = min(W, cx_pix + r + 1)
            y0 = max(0, cy_pix - r); y1 = min(H, cy_pix + r + 1)
            z = np.nanmedian(depth[y0:y1, x0:x1]) if (x1>x0 and y1>y0) else np.nan

            label = f"obj {i}"
            if np.isfinite(z) and self.have_cam_info:
                fx = self.cam_model.fx(); fy = self.cam_model.fy()
                cx0 = self.cam_model.cx(); cy0 = self.cam_model.cy()
                X = (cx_pix - cx0) * z / fx
                Y = (cy_pix - cy0) * z / fy   # REP 103: +Y down in optical frame
                label += f"  X={X:.3f} Y={Y:.3f} Z={z:.3f} m"
                # Also print to console
                rospy.loginfo_throttle(0.5, "obj %d @ camera_optical: X=%.3f Y=%.3f Z=%.3f m",
                                       i, X, Y, z)
            elif np.isfinite(z):
                label += f"  Z={z:.3f} m (no cam_info)"
            else:
                label += "  Z=nan"

            cv2.putText(vis, label, (x, max(0,y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Make a depth colormap panel
        depth_vis = depth.copy()
        zmin, zmax = np.nanpercentile(depth_vis, 2), np.nanpercentile(depth_vis, 98)
        zmin = max(0.1, float(zmin)); zmax = max(zmin+1e-3, float(zmax))
        depth_norm = (np.nan_to_num(depth_vis, nan=0.0) - zmin) / (zmax - zmin)
        depth_norm = np.clip(depth_norm, 0.0, 1.0)
        depth_color = cv2.applyColorMap((depth_norm*255).astype(np.uint8), cv2.COLORMAP_TURBO)

        for c in cnts:
            if cv2.contourArea(c) < self.min_area:
                continue
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(depth_color, (x,y), (x+w,y+h), (0,255,0), 2)

        stacked = np.hstack((vis, depth_color, cv2.cvtColor(mask_total, cv2.COLOR_GRAY2BGR)))
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(stacked, "bgr8"))
        except CvBridgeError:
            pass

if __name__ == "__main__":
    rospy.init_node("step4_rgbd1_object_xyz")
    HSVWithDepthXYZ()
    rospy.spin()