#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from pycoral.adapters import common
from pycoral.utils.edgetpu import make_interpreter


class YOLO_Detection(): 
    def __init__(self):
        # Subscriber
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.Detection, queue_size=1)
        
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

        # Confidence Score
        self.score_threshold = float(rospy.get_param("~conf_value", 0.6))

        # Model Path
        self.model_path = rospy.get_param("~weights", "best_int8_edgetpu.tflite")

        # TPU / TFLite setup --------------
        rospy.loginfo(f"[yolo_detection] initializing with model: {self.model_path}")
        self.interp = make_interpreter(self.model_path)
        self.interp.allocate_tensors()
        self.input_size = common.input_size(self.interp)   # (width, height)
        rospy.loginfo(f"[yolo_detection] model input size: {self.input_size}")

        # Variables -----------------------
        self.lastSign = None
        self.counter = 0
        self.busy = False  # Busy flag
        
        rospy.loginfo("[yolo_detection] node init complete")

    def cbTaskBusy(self, task: Bool):
        self.busy = bool(task.data)
        rospy.loginfo(f"[yolo_detection] /task/busy -> {self.busy}")

    def Detection(self, image_msg): 
        # Stop detection if busy
        if self.busy:
            rospy.loginfo_throttle(1.0, "[yolo_detection] busy=True, skipping frame")
            return
        
        # Decode image from topic
        rospy.loginfo_throttle(1.0, "[yolo_detection] callback triggered")
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image_input is None:
            rospy.logwarn("[yolo_detection] cv2.imdecode returned None; skipping frame")
            return
        
        ih, iw = cv_image_input.shape[:2]
        in_w, in_h = self.input_size
        rospy.logdebug(f"[yolo_detection] input frame size: {iw}x{ih}, model size: {in_w}x{in_h}")

        # Preprocess for EdgeTPU (uint8 RGB)
        resized = cv2.resize(cv_image_input, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        common.set_input(self.interp, rgb)

        # Run inference
        rospy.logdebug("[yolo_detection] invoking interpreter")
        self.interp.invoke()
        out_tensor = common.output_tensor(self.interp, 0)
        rospy.logdebug(f"[yolo_detection] raw output tensor shape: {out_tensor.shape}")

        # ---------------- YOLOv8 Postprocessing ----------------
        output = out_tensor[0]  # shape (N, C)
        xywh = output[:, 0:4]
        obj_conf = output[:, 4]
        cls_conf = output[:, 5:]
        cls_id = np.argmax(cls_conf, axis=1)
        cls_score = cls_conf[np.arange(len(cls_conf)), cls_id]
        scores = obj_conf * cls_score

        # Filter low confidence detections
        mask = scores > self.score_threshold
        xywh, scores, cls_id = xywh[mask], scores[mask], cls_id[mask]
        rospy.loginfo(f"[yolo_detection] filtered detections: {xywh.shape[0]} (thr={self.score_threshold:.2f})")

        boxes = []
        for (x, y, w, h, s, c) in zip(xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3], scores, cls_id):
            # YOLOv8 uses normalized xywh (0–1)
            x *= in_w
            y *= in_h
            w *= in_w
            h *= in_h
            x0 = int(x - w / 2)
            y0 = int(y - h / 2)
            x1 = int(x + w / 2)
            y1 = int(y + h / 2)
            boxes.append({'bbox': (x0, y0, x1, y1), 'score': float(s), 'cls': int(c)})

        # Optional: basic NMS to remove duplicates
        before_nms = len(boxes)
        boxes = self.non_max_suppression(boxes, iou_threshold=0.45)
        rospy.loginfo(f"[yolo_detection] boxes before NMS: {before_nms}, after NMS: {len(boxes)}")

        # --------------------------------------------------------

        vis = cv_image_input.copy()

        if not boxes:
            self.lastSign = None
            self.counter = 0
            rospy.logdebug("[yolo_detection] no boxes this frame")
        else:
            # Draw boxes
            for det in boxes:
                x0, y0, x1, y1 = det['bbox']
                score = det['score']
                cls = det['cls']
                label = self.CLASS.get(cls, str(cls))
                cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(vis, f"{label}:{score:.2f}", (x0, max(12, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Pick the best object (highest score)
            best_obj = max(boxes, key=lambda o: o['score'])
            sign = self.CLASS.get(best_obj['cls'], str(best_obj['cls']))
            rospy.loginfo(f"[yolo_detection] best detection: {sign} @ {best_obj['score']:.2f}")

            # Verify repeated detections
            if sign == self.lastSign:
                self.counter += 1
            else:
                self.counter = 1
                self.lastSign = sign

            rospy.loginfo(f"[yolo_detection] repeat counter: {self.counter}")
            if self.counter >= 3:
                self.pub_sign.publish(String(data=sign))
                self.busy = True
                self.counter = 0
                rospy.loginfo(f"[yolo_detection] Published sign '{sign}', setting busy=True")

        # Publish visualization frame
        try:
            msg = self.cvbridge.cv2_to_compressed_imgmsg(vis, dst_format='jpeg')
            self.pub_vis.publish(msg)
            rospy.loginfo_throttle(1.0, "[yolo_detection] vis frame published")
        except Exception as e:
            rospy.logwarn(f"[yolo_detection] failed to publish vis frame: {e}")

    # ---------------- Helper: Non-Maximum Suppression ----------------
    def non_max_suppression(self, boxes, iou_threshold=0.45):
        if not boxes:
            return []
        boxes = sorted(boxes, key=lambda x: x['score'], reverse=True)
        keep = []
        while boxes:
            current = boxes.pop(0)
            keep.append(current)
            boxes = [b for b in boxes if self.iou(current, b) < iou_threshold]
        return keep

    def iou(self, a, b):
        ax0, ay0, ax1, ay1 = a['bbox']
        bx0, by0, bx1, by1 = b['bbox']
        inter_x0 = max(ax0, bx0)
        inter_y0 = max(ay0, by0)
        inter_x1 = min(ax1, bx1)
        inter_y1 = min(ay1, by1)
        inter_area = max(0, inter_x1 - inter_x0) * max(0, inter_y1 - inter_y0)
        area_a = max(0, (ax1 - ax0)) * max(0, (ay1 - ay0))
        area_b = max(0, (bx1 - bx0)) * max(0, (by1 - by0))
        union = area_a + area_b - inter_area
        return inter_area / union if union > 0 else 0
    # -----------------------------------------------------------------

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('yolo_detection')
    node = YOLO_Detection()
    node.main()
