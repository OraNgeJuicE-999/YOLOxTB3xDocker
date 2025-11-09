import rospy, cv2
import numpy as np
from std_msgs.msg import UInt8, Float64, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
# from dynamic_reconfigure.server import Server
# from detect.cfg import DetectLaneParamsConfig

class DetectLane():
    def __init__(self):

        self.reliability_yellow_line = 100

        self.left_fit = None
        self.right_fit = None
        self.left_fitx = None
        self.right_fitx = None

        self.counter = 1
        self.cvBridge = CvBridge()

        self.lane_toggle = True
        self.white_toggle = True
        self.yellow_toggle = True

        # time sleep timer rate
        self.rate = rospy.Rate(10)

        # Getting params from the yaml files
        self.top_x = rospy.get_param("~top_x", 75)
        self.top_y = rospy.get_param("~top_y", 40)
        self.bottom_x = rospy.get_param("~bottom_x", 150)
        self.bottom_y = rospy.get_param("~bottom_y", 120)

        self.hue_white_l = rospy.get_param("~hue_white_l", 0)
        self.hue_white_h = rospy.get_param("~hue_white_h", 179)
        self.saturation_white_l = rospy.get_param("~saturation_white_l", 0)
        self.saturation_white_h = rospy.get_param("~saturation_white_h", 50)
        self.lightness_white_l = rospy.get_param("~lightness_white_l", 200)
        self.lightness_white_h = rospy.get_param("~lightness_white_h", 255)
        self.reliability_white_line = 100

        self.hue_yellow_l = rospy.get_param("~hue_yellow_l", 15)
        self.hue_yellow_h = rospy.get_param("~hue_yellow_h", 35)
        self.saturation_yellow_l = rospy.get_param("~saturation_yellow_l", 80)
        self.saturation_yellow_h = rospy.get_param("~saturation_yellow_h", 255)
        self.lightness_yellow_l = rospy.get_param("~lightness_yellow_l", 80)
        self.lightness_yellow_h = rospy.get_param("~lightness_yellow_h", 255)

        self.right_shift = rospy.get_param("~right_shift", 320)
        self.left_shift = rospy.get_param("~left_shift", 280)

        # Subscriber
        self.sub_bot_x = rospy.Subscriber('/detect/lane_bot_x', UInt8, self.cbBotX, queue_size=1)
        self.sub_top_y = rospy.Subscriber('/detect/lane_top_y', UInt8, self.cbTopY, queue_size=1)
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.cbImageProjection, queue_size=1)

        self.sub_lane_toggle = rospy.Subscriber('/detect/lane_toggle', Bool, self.cbLaneToggle, queue_size=1)
        self.sub_white_toggle = rospy.Subscriber('/detect/white_toggle', Bool, self.cbWhiteToggle, queue_size=1)
        self.sub_yellow_toggle = rospy.Subscriber('/detect/yellow_toggle', Bool, self.cbYellowToggle, queue_size=1)

        # Publisher
        self.pub_lane = rospy.Publisher('/detect/lane/compressed', CompressedImage, queue_size=1)
        self.pub_lane_center = rospy.Publisher('/control/lane', Float64, queue_size=1)

    def cbWhiteToggle(self, msg):
        self.white_toggle = msg.data

    def cbYellowToggle(self, msg):
        self.yellow_toggle = msg.data

    def cbBotX(self, msg):
        self.bottom_x = msg.data

    def cbTopY(self, msg):
        self.top_y = msg.data

    def cbLaneToggle(self, msg):
        self.lane_toggle = msg.data

    def cbImageProjection(self, msg_img):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        # compressed image
        np_image_original = np.frombuffer(msg_img.data, np.uint8)
        cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        if cv_image_original is None:
            return

        # raw image
        # cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        # 高斯濾波
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        # 鳥瞰圖轉換
        pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])
        h, status = cv2.findHomography(pts_src, pts_dst)
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        black = (0, 0, 0)
        cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        # 車道偵測
        cv_lane = np.copy(cv_image_homography)

        white_fraction, cv_white_lane = self.maskWhiteLane(cv_lane)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_lane)

        if cv_white_lane is None or cv_white_lane.size == 0:
            return
        if cv_yellow_lane is None or cv_yellow_lane.size == 0:
            return
        
        if yellow_fraction > 4000:
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

        if white_fraction > 4000:
            self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
            self.mov_avg_right = np.array([self.right_fit])
        # try:
        #     if yellow_fraction > 4000:
        #         self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
        #         self.mov_avg_left = np.array([self.left_fit])

        #     if white_fraction > 4000:
        #         self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
        #         self.mov_avg_right = np.array([self.right_fit])
        # except:
        #     if yellow_fraction > 4000:
        #         self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
        #         self.mov_avg_left = np.append(self.mov_avg_left,np.array([self.left_fit]), axis=0)

        #     if white_fraction > 4000:
        #         self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
        #         self.mov_avg_right = np.append(self.mov_avg_right,np.array([self.right_fit]), axis=0)

        MOV_AVG_LENGTH = 5

        try:
            self.left_fit = np.array([np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
           
            if self.mov_avg_left.shape[0] > 1000:
                self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]
        except:
            pass

        try:
            self.right_fit = np.array([np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

            if self.mov_avg_right.shape[0] > 1000:
                self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]
        except:
            pass

        self.make_lane(cv_lane, white_fraction, yellow_fraction)

    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        if fraction_num > 30000:
            if self.lightness_white_l < 250:
                self.lightness_white_l += 5
        elif fraction_num < 5000:
            if self.lightness_white_l > 50:
                self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line

        # rospy.loginfo("WHITE : %d", msg_white_line_reliability.data)

        return fraction_num, mask

    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        if fraction_num > 30000:
            if self.lightness_yellow_l < 250:
                self.lightness_yellow_l += 5
        elif fraction_num < 5000:
            if self.lightness_yellow_l > 50:
                self.lightness_yellow_l -= 5

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= 100:
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_white_yellow_reliability = UInt8()
        msg_white_yellow_reliability.data = self.reliability_yellow_line

        # rospy.loginfo("YELLOW : %d",msg_white_yellow_reliability.data)

        return fraction_num, mask

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 150 #100
        lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
        nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

        # Again, extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        lane_fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
           
        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # midpoint = np.int(histogram.shape[0] / 2)
        midpoint = histogram.shape[0] // 2


        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20

        # Set height of windows
        # window_height = np.int(img_w.shape[0] / nwindows)
        window_height = img_w.shape[0] // nwindows

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        x_current = lane_base

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Create empty lists to receive lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_lane_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_lane_inds) > minpix:
                x_current = int(np.mean(nonzerox[good_lane_inds]))

        # Concatenate the arrays of indices
        lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except:
            lane_fit = self.lane_fit_bef

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    def make_lane(self, cv_image, white_fraction, yellow_fraction):
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if yellow_fraction > 3000:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        if white_fraction > 3000:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
       
        self.is_center_x_exist = True

        if self.white_toggle == False:
            self.reliability_white_line = 0
        if self.yellow_toggle == False:
            self.reliability_yellow_line = 0

        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:  
            # if white_fraction > 3000 and yellow_fraction > 3000:
            #     centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
            #     pts = np.hstack((pts_left, pts_right))
            #     pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            #     cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(153, 0, 153), thickness=12)

            #     # Draw the lane onto the warped blank image
            #     cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            if white_fraction + 300 > yellow_fraction and (self.right_fitx is not None):
                centerx = np.subtract(self.right_fitx, self.right_shift)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(219,112,147), thickness=12)

            elif (yellow_fraction > 3000 and white_fraction < 3000) and (self.left_fitx is not None): 
            # else:
                
                centerx = np.add(self.left_fitx, self.left_shift) # 320
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(233, 203, 49), thickness=12)
            else: 
                self.is_center_x_exist = False
                # TODO: stop
                pass
        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50 and (self.left_fitx is not None):
            centerx = np.add(self.left_fitx, self.left_shift) # 320
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(37, 195, 26), thickness=12)

        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50 and (self.right_fitx is not None):
            centerx = np.subtract(self.right_fitx, self.right_shift)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            # cv2.polylines(color_warp_liright_fitx, 320)
            # pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(252, 174, 57), thickness=12)

        # else:
        #     self.is_center_x_exist = nes, np.int_([pts_center]), isClosed=False, color=(252, 174, 57), thickness=12)

        else:
            self.is_center_x_exist = False
            # TODO: stop
            pass

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.is_center_x_exist and centerx is not None:
        # publishes lane center
            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(350)
            if self.lane_toggle == True:
                self.pub_lane_center.publish(msg_desired_center)
            # rospy.loginfo(msg_desired_center)

        self.pub_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, "jpg"))
   
    def main(self):
       
        rospy.spin()
        # sleep for 10 Hz (0.2s)
        self.rate.sleep()

if __name__ == '__main__':

    rospy.init_node('detect_lane')
    node = DetectLane()
    node.main()