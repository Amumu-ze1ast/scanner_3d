import cv2
import numpy as np

img = cv2.imread('rod.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Click on the object to get HSV values
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"HSV at ({x},{y}): {hsv[y,x]}")

cv2.imshow('image', img)
cv2.setMouseCallback('image', mouse_callback)
cv2.waitKey(0)