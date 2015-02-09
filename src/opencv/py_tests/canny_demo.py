#!/usr/bin/python
from math import pi, sin, cos, sqrt
import cv
import cv2
import numpy as np

cam1 = cv.CaptureFromCAM(1)
#cam1 = cv2.VideoCapture(1)
cv.SetCaptureProperty(cam1, cv.CV_CAP_PROP_FRAME_WIDTH, 640)
cv.SetCaptureProperty(cam1, cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

cv.NamedWindow("1")
cv.NamedWindow("Edges")
cv.NamedWindow("Hough Lines")
cv.NamedWindow("Blurred")

with_edges = cv.CreateImage((640, 480), 8, 1)
gray = cv.CreateImage((640, 480), 8, 1)
temp = cv.CreateImage((640, 480), 8, 1)

# Thresholds for processing
low = 100
high = 300
lines = 0
while True:
  frame1 = cv.QueryFrame(cam1)
  #boolean,frame1 = cam1.read()
  print type(frame1)
  color_dst = cv.CreateImage(cv.GetSize(frame1), 8, 3)
  #color_dst = cv.CreateImage((640, 480), 8, 3)
  storage = cv.CreateMemStorage(0)
  cv.CvtColor(frame1, gray, cv.CV_BGR2GRAY)
  #`np_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
  cv.Smooth(gray, with_edges, cv.CV_GAUSSIAN, 7, 7, 0)
  #np_gray = np.array(gray, dtype="int32")
  #cv2.medianBlur(np_gray, 5)
  cv.Not(gray, with_edges)
  cv.Canny(gray, with_edges, low, high, 3)
  cv.CvtColor(with_edges, color_dst, cv.CV_GRAY2BGR)
  cv.ShowImage("Edges", with_edges)
  lines = cv.HoughLines2(with_edges, storage, cv.CV_HOUGH_PROBABILISTIC, 1, pi / 180, 80, 100, 10)
  for line in lines:
    if abs(line[0][0] - line[1][0]) < 10 or abs(line[0][1] - line[1][1]) < 10:
      cv.Line(color_dst, line[0], line[1], cv.CV_RGB(255, 0, 0), 3, 8)
  cv.ShowImage("Hough Lines", color_dst)
  temp = with_edges
  if cv.WaitKey(10) >= 0:
    break

cv.DestroyAllWindows()
