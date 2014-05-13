#!/usr/bin/python

import roslib
roslib.load_manifest('but_pr2_greeter')
import rospy

import threading

import numpy as np
import scipy as sp
from scipy.ndimage.filters import median_filter

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import message_filters
from camera_calibration.approxsync import ApproximateSynchronizer
from geometry_msgs.msg import PointStamped
import image_geometry

class FaceDetector:
  
  def __init__(self, debug=False, disable_depth = True, cam_ns = "/head_kinect"):

    self._debug = debug
    self._disable_depth = disable_depth
    
    self._img_topic = cam_ns + "/rgb/image_rect_color/compressed"
    # self._depth_topic = "/camera/depth_registered/hw_registered/image_rect_raw/compressed"
    self._depth_topic = cam_ns + "/depth_registered/hw_registered/image_rect_raw"
    self._info_topic = cam_ns + "/rgb/camera_info"    
  
    # self._img_sub = rospy.Subscriber(img_topic, CompressedImage, self.img_cb,  queue_size = 1)
    
    self._img_sub = message_filters.Subscriber(self._img_topic, CompressedImage)
    # self._depth_sub = message_filters.Subscriber(self._depth_topic, CompressedImage)
    self._depth_sub = message_filters.Subscriber(self._depth_topic, Image)
    self._info_sub = message_filters.Subscriber(self._info_topic, CameraInfo)
    
    self._point_pub = rospy.Publisher('nearest_face', PointStamped)
    
    self._img = None
    self._img_gray = None
    
    self._img_faces = None
    
    self._depth = None
    self._depth_vis = None
    
    self._cam_model = None
    
    self._face_det = cv2.CascadeClassifier('/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml')
    self._face_det2 = cv2.CascadeClassifier('/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_profileface.xml')
    
    if self._debug:
      
      rospy.loginfo("Ready (debug mode)")
      
      # self._timer = rospy.Timer(rospy.Duration(0.1), self.timer)
      self._display = threading.Thread(target=self.timer)
      self._display.daemon = True
      self._display.start()
      
    else:
      
      rospy.loginfo("Ready")

    if self._disable_depth:
      
      rospy.loginfo("Subscribing to: " + self._img_topic + " and " + self._info_topic + ".")

      self._ts = ApproximateSynchronizer(0.25, [self._img_sub, self._info_sub], 3)
      self._ts.registerCallback(self.kinect_cb)
      
    else:
      
      rospy.loginfo("Subscribing to: " + self._img_topic +  ", " + self._depth_topic + " and " + self._info_topic + ".")

      self._ts = ApproximateSynchronizer(0.25, [self._img_sub, self._depth_sub, self._info_sub], 3)
      self._ts.registerCallback(self.kinect_depth_cb)
  
  def get_face(self, image):
    
    np_arr = np.fromstring(image.data, np.uint8)
    self._img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    
    assert self._img is not None
    
    self._img_gray = cv2.cvtColor(self._img, cv2.COLOR_BGR2GRAY)
    self._img_gray = cv2.equalizeHist(self._img_gray)
    
    rects = self._face_det.detectMultiScale(self._img_gray, 1.6, 3, cv2.cv.CV_HAAR_SCALE_IMAGE, (30, 30))

    #print "faces: " + str(len(rects))
    
    if len(rects) == 0:

      return (False, None, None)

    #if len(rects) == 0:
      
    #  rects = self._face_det2.detectMultiScale(self._img_gray, 1.4, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20, 20))
      
    #  if len(rects) > 0:
        
    #    rospy.loginfo("backup detector worked")
        
    #  else:
        
    #    dbg_img = self._img_gray
    #    self._img_faces = dbg_img

#	print "backup failed"
        
 #       return (False, None, None)
      
    max_area = 0
    max_area_idx = None
    
    # find largest face
    for idx, val in enumerate(rects):
      
      if (val[2] * val[3]) > max_area:
        
        max_area_idx = idx
      
    if max_area_idx is not None:
      
      assert rects[max_area_idx][2] == rects[max_area_idx][3]
      
      size = rects[max_area_idx][2]
      
      #pt1 = (rects[max_area_idx][0] + size/4, rects[max_area_idx][1] + size/4)
      #pt2 = (rects[max_area_idx][0] + size - size/4, rects[max_area_idx][1] + size - size/4)
    
      pt1 = (rects[max_area_idx][0], rects[max_area_idx][1])
      pt2 = (rects[max_area_idx][0] + size, rects[max_area_idx][1] + size)
    
      if self._debug:
        
        dbg_img = self._img_gray
        cv2.rectangle(dbg_img, pt1, pt2, (127, 255, 0), 2)
        self._img_faces = dbg_img
        self._depth_vis = self._depth
        cv2.rectangle(self._depth_vis, pt1, pt2, (127, 255, 0), 2)
      
      #cx = rects[max_area_idx][0] + rects[max_area_idx][2] / 2
      #cy = rects[max_area_idx][1] + rects[max_area_idx][3] / 2
        
      #print "det"

      return (True, pt1, pt2)

    else:

      print "big error"

      return (False, None, None)
  
  def publish(self, header, pt1, pt2, depth):
  	
    pts = PointStamped()
      
    pts.header = header
    
    (cx, cy) = self.get_centres(pt1,pt2)
    
    ray = self._cam_model.projectPixelTo3dRay((cx, cy))
    pt = np.dot(ray, depth)
      
    pts.point.x = pt[0]
    pts.point.y = pt[1]
    pts.point.z = pt[2]

    #print pts
    
    self._point_pub.publish(pts)
  
  def kinect_cb(self, image, info):

    if self._img is None:
      
      rospy.loginfo("Kinect data received!")
      
    if self._cam_model is None:
      
      self._cam_model = image_geometry.PinholeCameraModel()
      self._cam_model.fromCameraInfo(info)
    
    (ret, pt1, pt2) = self.get_face(image)
    
    if not ret:
      
      return
     
    self.publish(image.header, pt1, pt2, 1.0)
  
  def get_centres(self,pt1,pt2):
  	
    cx = pt1[0] + ((pt2[0]-pt1[0])/2)
    cy = pt1[1] + ((pt2[1]-pt1[1])/2)
    
    return (cx, cy)
  
  def kinect_depth_cb(self, image, depth, info):
    
    if self._debug and self._img is None:
      
      rospy.loginfo("Kinect (depth) data received!")
    
    assert image.header.frame_id == depth.header.frame_id
    
    if self._cam_model is None:

      self._cam_model = image_geometry.PinholeCameraModel()
      self._cam_model.fromCameraInfo(info)
    
    (ret, pt1, pt2) = self.get_face(image)
    
    if not ret:
      
      return
    
    (cx, cy) = self.get_centres(pt1,pt2)
    
    # print "depth: " + str(len(depth.data))
    
    # np_arr_d = np.fromstring(depth.data, np.uint16)
    self._depth = np.fromstring(depth.data, np.uint16)
    self._depth.resize((480, 640))
    
    # self._depth = cv2.imdecode(np_arr_d, cv2.CV_LOAD_IMAGE_UNCHANGED)
    
    assert self._depth is not None

    dist_face = self._depth[pt1[0]:pt2[0], pt1[1]:pt2[1]]
    dist_face = median_filter(dist_face, 3) # TODO make filter size proportional to image size?
      
    ar = np.array([])
      
    for (x,y), value in np.ndenumerate(dist_face):
        
      if (np.isnan(value)) or (value == 0) or (value == 127):
          
        continue
        
      ar = np.append(ar, value)
        
        
    if len(ar) == 0:
        
       dbg_img = self._img_gray
       self._img_faces = dbg_img
        
       return
      
    dist = np.amin(ar) # min value
    
    self.publish(image.header, pt1, pt2, dist/1000.0)
          
  def timer(self):
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
      if self._img_faces is not None and self._depth_vis is not None:
        
        # TODO mutex?
        cv2.imshow('detected_faces', self._img_faces)
        cv2.imshow('depth', self._depth_vis)
        cv2.waitKey(1)
             
        r.sleep()
           

if __name__ == '__main__':
  
  rospy.init_node('but_pr2_face_detector')
  rospy.loginfo("PR2 FaceDetector")
  
  bpg = FaceDetector(debug=False)
  
  rospy.spin()
