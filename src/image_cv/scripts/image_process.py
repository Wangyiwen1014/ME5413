#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

ratio_w = 0.35  # 0.32
ratio_h = 0.48  # 0.45

# Camera focal length(pixel)
FOCAL_LENGTH = 554.254691191187  

# Box width(m)
BLOCK_WIDTH = 0.8  

Flag = 'False'
First_flag = 'False'
current_pose = None  # Current pose
initial_pose = None  # Initial pose 
initial_flag = 'False'
initial_point = {'x': 17, 'y': -4.5, 'yaw': 1.57}
bias_pose = None

angular_velocity_z = None  # Angular velocity in z
orientation_quaternion = None  # Pose quaternion
yaw = None

current_dir = os.path.dirname(os.path.realpath(__file__))
template_images = []
for i in range(1, 11):
    template_images.append(cv2.imread(os.path.join(current_dir, f'template_2/2_{i}.png'), cv2.IMREAD_GRAYSCALE))

def odometry_callback(msg):
    global current_pose, initial_flag, initial_pose, bias_pose
    current_pose = msg.pose.pose
    if initial_flag == 'False':
        initial_pose = current_pose
        initial_pose_info = f"Initial Position: x={initial_pose.position.x:.2f}, y={initial_pose.position.y:.2f}, z={initial_pose.position.z:.2f}"
        rospy.loginfo(initial_pose_info)
        bias_pose = Pose()
        bias_pose.position.x = initial_pose.position.x - initial_point['x']
        bias_pose.position.y = initial_pose.position.y - initial_point['y']
        rospy.loginfo(f"Bias Pose: x={bias_pose.position.x:.2f}, y={bias_pose.position.y:.2f}, z={bias_pose.position.z:.2f}")
        initial_flag = 'True'


def imu_callback(msg):
    global angular_velocity_z, orientation_quaternion, yaw
    angular_velocity_z = msg.angular_velocity.z
    orientation_quaternion = msg.orientation
    
    
    euler_angles = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
    yaw = euler_angles[2]  # Yaw angle
    #rospy.loginfo(f"Yaw angle: {yaw:.2f} radians")  # Output to terminal


    
def talker(distance_str):
    global Flag, current_pose, bias_pose
    if Flag == 'True' and current_pose is not None:
        pub = rospy.Publisher('signal', String, queue_size=10)
        position = current_pose.position
        
        position_info = f"Robot Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}"
        

        new_position = Point()
        new_position.x = position.x - bias_pose.position.x
        new_position.y = position.y - bias_pose.position.y
        new_position.z = position.z 
        
        new_position_info = f"Robot New Position: x={new_position.x:.2f}, y={new_position.y:.2f}, z={new_position.z:.2f}"
        
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            rospy.loginfo(distance_str)
            rospy.loginfo(position_info)
            rospy.loginfo(new_position_info)
            pub.publish(f"{distance_str}, {new_position_info}")
            Flag = 'False'  # Set Flag to 'False'
            rate.sleep()  
        

def image_callback(msg):
    global Flag
    global First_flag, yaw
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
        img_height, img_width = cv_image.shape
        target_gray = cv2.GaussianBlur(cv_image, (5, 5), 0)
        
        max_similarity = -1
        best_match_loc = None
        best_template_index = None
                
        for idx, template_image in enumerate(template_images):
            res = cv2.matchTemplate(target_gray, template_image, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)

            if max_val > max_similarity:
                max_similarity = max_val
                best_match_loc = max_loc
                best_template_index = idx
               
        threshold = 0.7
        if max_similarity >= threshold:
            template_image_shape = template_images[best_template_index].shape
            top_left = best_match_loc
            bottom_right = (top_left[0] + template_image_shape[1], top_left[1] + template_image_shape[0])
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 0, 255), 2)

            width = template_image_shape[1]
            height = template_image_shape[0]

            W = round(width / ratio_w, 2)
            H = round(height / ratio_h, 2)

            AVG = round((W + H) / 2, 2)

            yaw_threshold = 0.5 # 0.5
            
            if abs((bottom_right[0] + top_left[0]) / 2 - img_width / 2) < img_width * 0.05 and First_flag == 'False' and (((1.57 - yaw_threshold) < yaw < (1.57 + yaw_threshold))  or ((-1.57 - yaw_threshold) < yaw < (-1.57 + yaw_threshold)) or ((0 - yaw_threshold) < yaw < (0 + yaw_threshold))) or ((-3.14) < yaw < (-3.14 + yaw_threshold)) or ((3.14- yaw_threshold) < yaw < (3.14)):    # or ((-3.14 - yaw_threshold) < yaw < (-3.14 + yaw_threshold))
                distance = (BLOCK_WIDTH * FOCAL_LENGTH) / AVG
                distance_str = f"Distance: {distance:.2f} m"
                Flag = 'True'
                First_flag = 'True'
                
                talker(distance_str) 
                
                # Output on image
                cv2.putText(cv_image, "Target in Center", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(cv_image, distance_str, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                Flag = 'False'  
                
        # cv2.putText(cv_image, Flag, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(e)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)  # Subscribe odom
    rospy.Subscriber("/imu/data", Imu, imu_callback)  # Subscribe IMU data
    rospy.Subscriber("/front/image_raw", Image, image_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

