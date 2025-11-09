#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import rospkg
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'app'))
# from app.edgetpumodel import EdgeTPUModel
# from app.utils import get_image_tensor 
from utils import get_image_tensor
from edgetpumodel import EdgeTPUModel

class YOLO_Detection(): 
    def __init__(self):
        # Subscriber
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.Detection, queue_size=1)
        
        # Publishers
        self.pub_sign = rospy.Publisher('/yolo/sign_detection', String, queue_size=1)
        self.pub_vis  = rospy.Publisher('/yolo/vis/compressed', CompressedImage, queue_size=1)
        
        # Task busy flag
        self.task_busy_toggle = rospy.Subscriber('/task/busy', Bool, self.cbTaskBusy, queue_size=1)

        # Class map (must match your model training classes & order)
        self.CLASS = {
            0: "Construction",
            1: "Intersection",  
            2: "Left", 
            3: "Right"
        }
        self.published_sign = []

        self.cvbridge = CvBridge()
        
        # Params
        self.conf_value = float(rospy.get_param("~conf_value", 0.6))
        self.image_size = int(rospy.get_param("~image_size", 480))
        self.model_path = rospy.get_param("~weights", "best_full_integer_quant_edgetpu.tflite")  # EdgeTPU-compiled .tflite
        pkg = rospkg.RosPack().get_path('yolo_detection')
        self.names_path = rospy.get_param('~data', os.path.join(pkg, 'data.yaml'))

        self.model = EdgeTPUModel(self.model_path, self.names_path,
                                  conf_thresh=self.conf_value, iou_thresh=0.45, v8=True)
        self.input_size = self.model.get_image_size()  # (H, W) or (W, H); wrapper defines it

        # Warm-up (optional)
        dummy = (255*np.random.random((3, *self.input_size))).astype(np.uint8)
        self.model.forward(dummy)

        self.lastSign = None
        self.counter = 0
        self.busy = False
    
    def cbTaskBusy(self, task: Bool):
        self.busy = bool(task.data)

    def Detection(self, image_msg: CompressedImage):
        if self.busy:
            return
        
        # Decode compressed image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Preprocess to model input (letterbox etc.)
        # get_image_tensor should return net_image in the layout/type your EdgeTPUModel expects
        full_image, net_image, pad = get_image_tensor(frame, self.input_size[0])

        # Inference on Edge TPU
        pred = self.model.forward(net_image)     # typically returns [det] where det is Nx6: [x1,y1,x2,y2,conf,cls]
        det = pred[0] if isinstance(pred, (list, tuple)) else pred

        # If your wrapper has a built-in drawer, you can call:
        # vis_img = self.model.process_predictions(det, full_image, pad, return_img=True)
        # Otherwise, do a simple draw here:
        if det is None or len(det) == 0:
            # No detections
            self.lastSign = None
            self.counter = 0
            # still publish visualization (original frame) if you want
            vis_img = full_image
        else:
            # det expected shape: [N, 6] -> (x1,y1,x2,y2,conf,cls)
            # If your wrapper returns a different format, adapt the indexing below.
            boxes  = det[:, 0:4]
            confs  = det[:, 4]
            clses  = det[:, 5].astype(int)

            # Choose highest-confidence detection
            idx = int(np.argmax(confs))
            best_cls = int(clses[idx])
            sign = self.CLASS.get(best_cls, None)

            # Debounce logic (3 consecutive frames)
            if sign == self.lastSign:
                self.counter += 1
            else:
                self.counter = 1
                self.lastSign = sign

            # Simple visualization (draw only the top-1 box)
            x1, y1, x2, y2 = boxes[idx].astype(int)
            vis_img = full_image.copy()
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{sign if sign else best_cls}: {confs[idx]:.2f}"
            cv2.putText(vis_img, label, (x1, max(0, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            if sign and self.counter >= 3:
                if sign not in self.published_sign:
                    self.pub_sign.publish(String(data=sign))
                    self.busy = True
                    self.published_sign.append(sign)
                    
                    self.counter = 0
                    rospy.loginfo(f"Published sign: {sign}, busy set True")
                

        # Publish visualization as CompressedImage
        vis_msg = self.cvbridge.cv2_to_compressed_imgmsg(vis_img, dst_format='jpg')
        self.pub_vis.publish(vis_msg)

        # Optional: log timing from EdgeTPUModel if available
        try:
            tinf, tnms = self.model.get_last_inference_time()
            # rospy.loginfo(f"Frame done in {tinf + tnms:.6f} s (inference {tinf:.6f} + nms {tnms:.6f})")
        except Exception:
            pass

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('yolo_detection')
    node = YOLO_Detection()
    node.main()

