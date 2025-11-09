#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class LedControlGPIO:
    """
    ROS node that listens to /yolo/sign_detection (std_msgs/String)
    and blinks an RGB LED via RPi.GPIO based on the detected sign.
    """

    # BCM pin mapping (change these if your wiring differs)
    PINS = {
        "R": 17,  # physical pin 11
        "G": 27,  # physical pin 13
        "B": 22,  # physical pin 15
    }

    # Map each sign to (R, G, B) state (1 = ON)
    COLORS = {
        "Left":     (1, 1, 0),  # yellow
        "Right":    (0, 1, 0),  # green
        "Intersection":  (0, 0, 1),  # blue
        "Construction":  (1, 0, 0),  # red
    }

    def __init__(self):
        self.blink_sleep = 10

        # Setup GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in self.PINS.values():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # ROS setup
        self.sub = rospy.Subscriber('/yolo/sign_detection', String, self.cb_detected_sign, queue_size=5)
        rospy.on_shutdown(self._cleanup)

    def _set_color(self, rgb):
        r, g, b = rgb
        GPIO.output(self.PINS["R"], GPIO.HIGH if r else GPIO.LOW)
        GPIO.output(self.PINS["G"], GPIO.HIGH if g else GPIO.LOW)
        GPIO.output(self.PINS["B"], GPIO.HIGH if b else GPIO.LOW)

    def _off(self):
        for p in self.PINS.values():
            GPIO.output(p, GPIO.LOW)

    def cb_detected_sign(self, msg):
        sign = msg.data.strip()
        if sign not in self.COLORS:
            rospy.logwarn(f"Unknown sign: {sign}")
            return

        rospy.loginfo(f"Detected sign: {sign}, blinking LED.")
        try:
            self._set_color(self.COLORS[sign])
            rospy.sleep(self.blink_sleep)
        finally:
            self._off()

    def spin(self):
        rospy.spin()

    def _cleanup(self):
        self._off()
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('led_control')
    node = LedControlGPIO()
    node.spin()
