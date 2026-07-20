#!/usr/bin/env python3
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

WIN = "tuner"

class HSVTunerNode:
    def __init__(self):
        rospy.init_node("hsv_tuner")
        self.bridge = CvBridge()
        self.gui_ready = False

        # Subscribe AFTER we init node (fine either way)
        # self.sub = rospy.Subscriber("/rgbd_camera1/rgb/image_raw", Image, self.cb, queue_size=1)
        self.sub = rospy.Subscriber("robot2_turntable/rgbd_camera1/rgb/image_raw", Image, self.cb, queue_size=1)


        rospy.loginfo("HSV tuner running. Use sliders to adjust thresholds.")

    def _setup_gui_once(self):
        if self.gui_ready:
            return
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 1200, 500)

        # show a tiny dummy frame once so Qt creates the window
        dummy = np.zeros((10, 10, 3), np.uint8)
        cv2.imshow(WIN, dummy)
        cv2.waitKey(1)

        # Create trackbars
        for name, val, mx in [
            ("H_low", 0, 180), ("S_low", 0, 255), ("V_low", 0, 255),
            ("H_high", 180, 180), ("S_high", 255, 255), ("V_high", 255, 255)
        ]:
            cv2.createTrackbar(name, WIN, val, mx, lambda x: None)

        cv2.createTrackbar("MinArea", WIN, 800, 20000, lambda x: None)
        self.gui_ready = True

    def _get_tb(self, name, default):
        # Safely read a trackbar (avoid NULL-window errors)
        try:
            return cv2.getTrackbarPos(name, WIN)
        except cv2.error:
            return default

    def cb(self, msg):
        # Make sure GUI exists before using trackbars
        self._setup_gui_once()

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Read sliders (with safe defaults in case GUI not ready yet)
        hL = self._get_tb("H_low", 0)
        sL = self._get_tb("S_low", 0)
        vL = self._get_tb("V_low", 0)
        hH = self._get_tb("H_high", 180)
        sH = self._get_tb("S_high", 255)
        vH = self._get_tb("V_high", 255)
        min_area = self._get_tb("MinArea", 800)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (hL, sL, vL), (hH, sH, vH))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vis = frame.copy()
        idx = 1
        for c in cnts:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx, cy = x + w // 2, y + h // 2
            cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(vis, f"obj {idx}", (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            idx += 1

        stacked = np.hstack((vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.imshow(WIN, stacked)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print(f"lower: [{hL},{sL},{vL}]  upper: [{hH},{sH},{vH}]  min_area: {min_area}")
        elif key in (27, ord('q')):  # ESC or q
            rospy.signal_shutdown("User quit")
            cv2.destroyAllWindows() 

if __name__ == "__main__":
    HSVTunerNode()
    rospy.spin()