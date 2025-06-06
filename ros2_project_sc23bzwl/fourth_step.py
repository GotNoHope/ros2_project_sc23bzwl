# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal
import time
import os
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from math import sin, cos

# os.environ["DISPLAY"] = ":0"

# Predefined box coordinates (based on grid layout)
GREEN_BOX = (-8.0, -2.0)
RED_BOX = (0.0, 9.0)
BLUE_BOX = (-8.0, 6.0) # 1m before the blue box 
SCAN_POINT = (6.0, 4.0)  # midpoint from which to scan left/right

class Robot(Node):
    
    
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session


        # Initialise any flags that signal a colour has been detected (default to false)


        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation_complete = False
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use

        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        self.green_found = False
        self.blue_found = False
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.rate = self.create_rate(10)  # 10 Hz
        self.too_close = False

    def send_navigation_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2.0)

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation result received')
        self.navigation_complete = True


    def callback(self, data):
        print("Image Received~")

        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(1)
        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        green_image = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        red_image = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        mask_image = cv2.bitwise_or(green_image,red_image)
        
        final_image = cv2.bitwise_and(image,image,mask = mask_image)

        
        
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(green_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        self.green_found = False
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 100:
                self.green_found = True
                self.too_close = area > 30000
                (x, y), r = cv2.minEnclosingCircle(c)
                cv2.circle(image, (int(x), int(y)), int(r), (0, 255, 0), 2)

        blue_contours, _ = cv2.findContours(blue_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.blue_found = False
        if len(blue_contours) > 0:
            c_blue = max(blue_contours, key=cv2.contourArea)
            if cv2.contourArea(c_blue) > 100:
                self.blue_found = True
                (x, y), r = cv2.minEnclosingCircle(c_blue)
                cv2.circle(image, (int(x), int(y)), int(r), (255, 0, 0), 2)

            #Moments can calculate the center of the contour
            #M = cv2.moments(c)
            
            #cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            print(cv2.contourArea(c))
            if cv2.contourArea(c) > 100: #<What do you think is a suitable area?>

                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 

                cv2.circle(image,center,radius,(255,255,0) ,1)

                # Then alter the values of any flags
                self.green_found = True
            else:
                self.green_found = False   

        #if the flag is true (colour has been detected)
            #print the flag or colour to test that it has been detected
            #alternatively you could publish to the lab1 talker/listener
        if self.green_found == True:
                print("Green detected")
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('threshold_Feed2',cv2.WINDOW_NORMAL) 
        cv2.imshow('threshold_Feed2', image)
        cv2.resizeWindow('threshold_Feed2',320,240)
        cv2.waitKey(3)
        
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler


        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        
        #hsv_colour1_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour1_upper = np.array([<Hue value> + self.sensitivity, 255, 255])
        
        #hsv_colour2_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour2_upper = np.array([<Hue value> + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image


        # Filter out everything but a particular colour using the cv2.inRange() method


        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter


        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

        # Loop over the contours
        #if len(greencontours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            #c = max(<contours>, key=cv2.contourArea)


            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            #if cv2.contourArea(c) > x #<What do you think is a suitable area?>:
                # Alter the value of the flag


        #Check if a flag has been set = colour object detected - follow the colour object
        if self.green_found == True:
            if cv2.contourArea(c) > 30000:
                # Too close to object, need to move backwards
                print("backward")
                self.too_close = True
                #self.walk_backward()
            elif cv2.contourArea(c) <= 30000 :
                print("forward")
                self.too_close = False
                # Too far away from object, need to move forwards
                #self.walk_forward()
        else:
            self.stop()
            
            # Be sure to do this for the other colour as well
            # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

        # Publish moves

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s
        start_time = time.time()
        while time.time() - start_time <2.0:  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            print("Moving forward")
            time.sleep(0.1)

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        start_time = time.time()
        while time.time() - start_time <2.0:  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            print("Backing up")
            time.sleep(0.1)

    def rotate_left(self, duration=2.0):
        desired_velocity = Twist()
        desired_velocity.angular.z = 0.5  # Positive Z for counter-clockwise rotation
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(desired_velocity)
            print("Rotating left")
            time.sleep(0.1)
            
    def rotate_in_place(self, duration=5.0, speed=0.5):
        twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        twist = Twist()
        twist.angular.z = speed

        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            twist_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        twist.angular.z = 0.0
        twist_pub.publish(twist)
        
        
    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)
        print("Stopping")



# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    
    print ("Robot is moving..")
    
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    rclpy.spin(robot)
    
    
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()
    
    try:
        # Step 1: Go to scanning point (center of map)
        robot.send_navigation_goal(6.0, 4.0, 0.0)  # Example scan point
        while not robot.navigation_complete and rclpy.ok():
            rclpy.spin_once(robot)
        robot.navigation_complete = False
        time.sleep(1)

        # Step 2: Rotate in place to scan for green/red
        robot.rotate_in_place(duration=5.0)
        time.sleep(2)  # Give camera time

        # Step 3: Go to detected colour box
        if robot.green_found:
            print("Navigating to GREEN box...")
            robot.send_navigation_goal(-8.0, -2.0, 0.0)  
        elif robot.red_found:
            print("Navigating to RED box...")
            robot.send_navigation_goal(0.0, 9.0, 0.0)  
            print("No red or green found. Aborting.")
            robot.stop()
            return

        while not robot.navigation_complete and rclpy.ok():
            rclpy.spin_once(robot)
        robot.navigation_complete = False
        time.sleep(1)

        # Step 4: Rotate again to detect next box
        robot.rotate_in_place(duration=5.0)
        time.sleep(2)

        if not robot.blue_found:
            # If second color (green or red) wasn't visited yet
            if robot.green_found:
                robot.send_navigation_goal(-8.0, -2.0, 0.0)
            else:
                robot.send_navigation_goal(0.0, 9.0, 0.0)
            while not robot.navigation_complete and rclpy.ok():
                rclpy.spin_once(robot)
            robot.navigation_complete = False
            time.sleep(1)

        # Step 5: Final spin to find blue
        robot.rotate_in_place(duration=5.0)
        time.sleep(2)

        if robot.blue_found:
            print("Blue found. Navigating close...")
            robot.send_navigation_goal(-8.0, 6.0, 0.0)  # Stop 1m short
            while not robot.navigation_complete and rclpy.ok():
                rclpy.spin_once(robot)
        else:
            print("Blue not detected. Task incomplete.")

        robot.stop()
        print("Mission complete.")

    except ROSInterruptException:
        robot.stop()
        # pass
# Check if the node is executing in the main path

    cv2.destroyAllWindows()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()





# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:~/ros2_ws/src/ros2_project_sc23bzwl/map