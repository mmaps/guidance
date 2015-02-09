#!/usr/bin/python
from math import pi, sin, cos
import cv
import cv2

width=640
height=480

cam1 = cv.CaptureFromCAM(1)
cv.SetCaptureProperty(cam1, cv.CV_CAP_PROP_FRAME_WIDTH, width)
cv.SetCaptureProperty(cam1, cv.CV_CAP_PROP_FRAME_HEIGHT, height)

cv.NamedWindow("Edges")
cv.NamedWindow("Lines")

edges = cv.CreateImage((width, height), 8, 1)
color_edges = cv.CreateImage((width, height), 8, 1)
lines = cv.CreateImage((width, height), 8, 1)
gray = cv.CreateImage((width, height), 8, 1)

canny_low = 10
canny_hi = 100
lines = 0

cv.CreateMemStorage(0)

while True:
  frame = cv.QueryFrame(cam1)
  cv.CvtColor(frame, gray, cv.CV_BGR2GRAY)
  cv.Smooth(gray, edges, cv.CV_GAUSSIAN, 7, 7, 0)
  cv.Not(gray, edges)
  cv.Canny(frame, edges, canny_low, canny_hi, 3)
  cv.CvtColor(edges, color_edges, cv.CV_GRAY2BGR)
  lines = cv.HoughLines2(edges, storage, cv.CV_HOUGH_STANDARD, 1, pi / 180, 100, 0, 0)
  for (rho, theta) in lines[:100]:
    a=cos(theta)
    b=sin(theta)
    x0=a*rho
    y0=b*rho
    pt1=(cv.Round(x0+1000*(-b)), cv.Round(y0+1000*(a)))
    pt2=(cv.Round(x0-1000*(-b)), cv.Round(y0-1000*(a)))
    cv.Line(color_edges, pt1, pt2, cv.RGB(255,0,0), 3, 8)
  cv.ShowImage("Edges", edges)
  cv.ShowImage("Lines", lines)
  if cv.WaitKey(0):
    break

cv.DestroyAllWindows()
