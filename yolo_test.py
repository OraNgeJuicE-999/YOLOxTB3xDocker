#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from ultralytics import YOLO
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class YOLO_Detection(): 
    def __init__(self):
        # Subscriber
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.Detection, queue_size = 1)
        
        # Publisher
        self.pub_sign = rospy.Publisher('/yolo/sign_detection', String, queue_size=1)
        self.pub_vis  = rospy.Publisher('/yolo/vis/compressed', CompressedImage, queue_size=1)
        
        # Check whether to activate yolo detect
        self.task_busy_toggle = rospy.Subscriber('/task/busy', Bool, self.cbTaskBusy, queue_size=1)

        # Class YAML
        self.CLASS = {
            0: "Construction",
            1: "Intersection",  
            2: "Left", 
            3: "Right"
        }

        # CV-Bridge
        self.cvbridge = CvBridge()

        # YOLO model param
        # self.model = YOLO("/root/ws/models/best.pt") # Pre-trained model
        # Confidence Score
        self.conf_value = float(rospy.get_param("~conf_value", 0.6))
        # Image Size
        self.image_size = int(rospy.get_param("~image_size", 640))

        self.model_path = rospy.get_param("~weights", "best.pt")
        
        self.model = YOLO(self.model_path)
        
        # Variables
        self.lastSign = None
        
        self.counter = 0

        # Busy Flag, True when one task is operating
        self.busy = False
    
    def cbTaskBusy(self, task: Bool):
        self.busy = bool(task.data)

    def Detection(self, image_msg): 
        # Check if busy is True, if True stop the loop
        
        # if self.busy:
        #     return
        
        # Obtain the image for YOLO (Compressed Image)
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image_input is None:
            return
        
        # cv2 captured values    
        # Perform object detection
        results = self.model(cv_image_input, conf=self.conf_value, imgsz=self.image_size)
        if not results:
            return
        
        r = results[0]
        vis = r.plot()
        vis_msg = self.cvbridge.cv2_to_compressed_imgmsg(vis, dst_format='jpg')
        self.pub_vis.publish(vis_msg)

        if not r.boxes:
            # If r does not have a value
            self.lastSign = None
            self.counter = 0

        else:
            # Obtain all conf and classes
            conf = r.boxes.conf.cpu().numpy()
            classes = r.boxes.cls.cpu().numpy().astype(int)
            
            # Find the highest conf's index
            highest_conf = np.argmax(conf)
            
            # Return the highest conf cls
            bestCls = classes[highest_conf]

            # Retrieve the index, and convert into the string
            sign = self.CLASS[int(bestCls)]

            # Verify
            if sign == self.lastSign:
                self.counter += 1
            else:
                # Reset the counter to 1
                self.counter = 1
                self.lastSign = sign
            
            if self.counter >= 3:
                # Publish after repeating sign
                self.pub_sign.publish(String(data=sign))
                self.busy = True
                self.counter = 0
                rospy.loginfo(self.busy)
                
                # rospy.sleep(10)
            
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('yolo_detection')
    node = YOLO_Detection()
    node.main()

                