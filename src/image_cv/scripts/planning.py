#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import subprocess
import os
import signal

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# 定义路径的角点坐标和朝向
points = [    
    {'x': 17, 'y': -4.5, 'yaw': 1.57},  
    {'x': 17, 'y': 1.4, 'yaw': 1.57},  
    {'x': 7.2, 'y': 1.4, 'yaw': -3.14},  
    {'x': 7.2, 'y': -6.7, 'yaw': -1.57},   
    {'x': 15.4, 'y': -6.7, 'yaw': 0.0}  
]

# 需要行走的路径索引，基于点的索引
paths = [(0, 1), (1, 2), (2, 3), (3, 4)]


# 初始化ROS节点
rospy.init_node('navigate_through_points')

# 创建move_base动作客户端
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# 创建子进程对象
image_process_proc = None

# 是否在下一个点停下
stop_at_next_point = False
current_path_index = 0  # Track current path index
First_flag = 'False'

# 话题回调函数，接收停止信号
def stop_callback(msg):
    global stop_at_next_point, current_path_index, First_flag
    # 检查消息是否包含距离信息
    if 'Distance' in msg.data and First_flag == 'False':
        try:
            # 提取距离信息
            parts = msg.data.split(',')
            distance_str = parts[0].split(':')[1].strip() 
            distance = float(distance_str.split()[0])
            # 提取小车当前位置信息
            x_pos_str = parts[1].split('=')[1].strip()
            y_pos_str = parts[2].split('=')[1].strip()
            x_pos = float(x_pos_str.split()[0])
            y_pos = float(y_pos_str.split()[0])
            # 根据路径段和方块的相对位置调整坐标
            X = 0.4 #0.8
            # distance = abs(distance)
            
            if current_path_index == 0:  # 0-1路径
                target_x = x_pos - (distance - X)
                target_y = y_pos
                yaw = 1.57
            elif current_path_index == 1:  # 1-2路径
                target_x = x_pos
                target_y = y_pos - (distance - X)
                yaw = -3.14
            elif current_path_index == 2:  # 2-3路径
                target_x = x_pos + (distance - X)
                target_y = y_pos
                yaw = -1.57
            elif current_path_index == 3:  # 3-4路径
                target_x = x_pos
                target_y = y_pos + (distance - X)
                yaw = 0
                
            rospy.loginfo(f"Calculating block position to navigate: x={target_x}, y={target_y}, yaw={yaw}")

            # 导航到计算出的方块位置
            navigate_to_block(target_x, target_y, yaw)
            stop_at_next_point = True
            
            First_flag = 'True'

        except ValueError as e:
            rospy.logerr(f"Error parsing message: {msg.data} | Error: {e}")
   


        
def navigate_to_block(x, y, yaw):
    """导航到计算出的目标方块位置，并输出日志信息"""
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    client.send_goal(goal)
    client.wait_for_result()
    
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Successfully reached the block!")
    else:
        rospy.loginfo("Failed to reach the block.")
   
def navigate_to_point(x, y, yaw):
    """导航到预定路径点，并输出日志信息"""
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    client.send_goal(goal)
    client.wait_for_result()
    
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Successfully reached point {x}, {y}")
    else:
        rospy.loginfo("Failed to reach the point.")
         
         
#def odometry_callback(msg):
    #global current_pose
    #current_pose = msg.pose.pose
    
    
# 订阅停止话题
rospy.Subscriber('/signal', String, stop_callback)

navigate_to_point(points[0]['x'], points[0]['y'], points[0]['yaw'])
rospy.sleep(0.5)

# 位置校正
#current_pose = None  # 用于存储当前机器人的位置和姿态信息
#rospy.Subscriber("/odometry/filtered", String, odom_callback)

    
# 定义目标点的位置和朝向
for idx, (start, end) in enumerate(paths):
    current_path_index = idx
    # 只在开始时启动图像处理脚本
    if idx == 0 and image_process_proc is None:
        image_process_path = os.path.join(os.path.dirname(__file__), 'image_process.py')
        image_process_proc = subprocess.Popen(['python3', image_process_path])
        
    # 导航到预定路径点并输出日志
    rospy.sleep(0.5)
    navigate_to_point(points[end]['x'], points[end]['y'], points[end]['yaw'])
    rospy.sleep(0.5)  # 稍微延迟，确保日志信息的输出有序
    if stop_at_next_point:
        rospy.loginfo("Detected object, stopped to calculate block position")
        break  # 检测到物体后，跳出循环
    

# 关闭节点时同时关闭子进程
def shutdown_hook():
    rospy.loginfo("Shutting down...")
    if image_process_proc:
        image_process_proc.send_signal(signal.SIGINT)


rospy.on_shutdown(shutdown_hook)

rospy.spin()

