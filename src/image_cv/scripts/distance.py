#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

# 相机焦距（以像素为单位）
FOCAL_LENGTH = 554.254691191187  # 请根据实际情况替换这个值

# 方块的实际宽度（以米为单位）
BLOCK_WIDTH = 0.8  # 请根据实际情况替换这个值

def calculate_distance(pixel_width):
    # 使用相似三角形计算距离
    distance = (BLOCK_WIDTH * FOCAL_LENGTH) / pixel_width
    rospy.loginfo(f"Calculated distance: {distance} meters")

def block_width_callback(data):
    # 从话题中接收像素宽度
    pixel_width = data.data
    # 计算距离
    calculate_distance(pixel_width)

def distance_node():
    # 初始化ROS节点
    rospy.init_node('distance_calculation_node')

    # 订阅 block_width 话题
    rospy.Subscriber('block_width', Int32, block_width_callback)

    # 保持程序持续运行直到被关闭
    rospy.spin()

if __name__ == '__main__':
    distance_node()

