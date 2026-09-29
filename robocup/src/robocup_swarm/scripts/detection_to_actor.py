#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""话题适配器：将 /swarm/detection 转发为官方 /actor_*_info 格式。

官方 ros_actor_cmd_pose_plugin 订阅 /actor_{target_id}_info，
原项目使用 /swarm/detection 发布检测结果。

本节点同时：
1. 订阅 /swarm/detection (TargetDetection)
2. 发布 /actor_{target_id}_info (PoseStamped) 供官方评分系统使用

用法：
  rosrun robocup_swarm detection_to_actor.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
from robocup_swarm.msg import TargetDetection


class DetectionToActor(object):
    def __init__(self):
        # 存储每个 target_id 对应的最新检测位置
        self._latest_detection = {}  # target_id -> (x, y, timestamp)

        # 订阅原始检测话题
        rospy.Subscriber("/swarm/detection", TargetDetection, self._detection_cb)

        # 为每个可能的目标 ID 创建发布者（动态创建）
        # target_id 格式：target_0, target_1, ..., target_5
        self._pubs = {}

        rospy.loginfo("[detection_to_actor] 启动，监听 /swarm/detection -> 转发到 /actor_*_info")

    def _detection_cb(self, msg):
        """接收检测消息，转发为 PoseStamped"""
        target_id = msg.target_id

        # 记录最新检测位置和时间
        self._latest_detection[target_id] = {
            'x': msg.x,
            'y': msg.y,
            'time': msg.header.stamp
        }

        # 获取或创建对应的话题发布者
        # actor_0_info, actor_1_info, ... (去掉 "target_" 前缀)
        actor_id = target_id.replace("target_", "actor_")
        topic_name = f"/{actor_id}_info"

        if topic_name not in self._pubs:
            self._pubs[topic_name] = rospy.Publisher(
                topic_name, PoseStamped, queue_size=10)
            rospy.loginfo(f"[detection_to_actor] 创建发布者: {topic_name}")

        # 构造 PoseStamped 消息（官方插件期望的格式）
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"  # 确保是地图坐标系
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0.0  # 目标在地面，z=0
        pose.pose.orientation.w = 1.0  # 单位四元数

        # 发布
        self._pubs[topic_name].publish(pose)


if __name__ == "__main__":
    rospy.init_node("detection_to_actor", anonymous=True)
    node = DetectionToActor()
    rospy.spin()
