#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

def move_base_result_callback(msg):
    if msg.text == "Goal reached.":
        rospy.loginfo("Goal reached. Starting planning...")
        # 在这里启动执行planning.py的内容，可以使用subprocess模块
        subprocess.Popen(['rosrun', 'image_cv', 'planning.py'])

def move_base_result_listener():
    rospy.init_node('move_base_result_listener', anonymous=True)
    rospy.Subscriber('/move_base/result', String, move_base_result_callback)
    rospy.spin()

def on_button_clicked(msg):
    if msg.data == "Button 2 clicked":
        rospy.loginfo("Button 2 clicked. Starting move_base_result_listener...")
        move_base_result_listener()

if __name__ == '__main__':
    rospy.init_node('button_listener', anonymous=True)
    rospy.Subscriber('/button_2_clicked', String, on_button_clicked)
    rospy.spin()

