#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把落在建筑物(black_box)内的 actor 挪到建筑外的空旷点。

为什么需要：
  官方 control_actor.py 在选新目标时会做路径碰撞检查（101 个路径点 × obstacle.txt
  里 2812 个障碍点，阈值 1.5 m）。actor_4(-32,-27) 和 actor_5(-35,-28) 的 init_pose
  恰好落在 black_box [[-35.4,-31.4],[-29.4,-13.4]] 内，从建筑内部出发的**任何**路径
  都会撞障碍 → 永远选不出合法点 → 这两个 actor 从不发布 cmd_motion。
  而官方 gazebo actor 插件在 GET_CMD_FLAG == false 时会每帧执行
  SetWorldPose(init_pose)，于是 actor 被死死钉在建筑里，形成死锁。

做法（不改任何官方文件）：
  1. 先给这些 actor 发一条 /actor_<i>/cmd_motion，让插件的 GET_CMD_FLAG 置位，
     插件就不再往 init_pose 写位姿了；
  2. 目标点选建筑外的空旷点（不在任何 black_box ±1.0 m 内，且距最近障碍 > 3 m）；
  3. 以 v=1.5 m/s 持续发这个目标，由 actor_motion_bridge.py 把 actor 真正带过去。

actor_motion_bridge.py 必须先启动。
"""
import ast
import math
import time

import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
from gazebo_msgs.srv import GetModelState

NUM_ACTOR = 6
BOX_FILE = '/root/XTDrone/robocup/black_box.txt'
OBS_FILE = '/root/XTDrone/robocup/obstacle.txt'
SPEED = 1.5
MAX_WAIT = 40.0          # 单个 actor 最多搬运这么久
ARRIVE = 2.0             # 距离目标小于这个值就算到了


def load_data():
    boxes = ast.literal_eval(open(BOX_FILE).read().strip())
    obs = []
    for line in open(OBS_FILE):
        p = line.split()
        if len(p) >= 2:
            try:
                obs.append((float(p[0]), float(p[1])))
            except ValueError:
                pass
    return boxes, obs


def make_clear_fn(boxes, obs, box_margin=1.0, obs_clear=3.0):
    def in_box(x, y):
        return any(b[0][0] - box_margin <= x <= b[0][1] + box_margin and
                   b[1][0] - box_margin <= y <= b[1][1] + box_margin
                   for b in boxes)

    def clear(x, y):
        if in_box(x, y):
            return False
        return all(math.hypot(o[0] - x, o[1] - y) > obs_clear for o in obs)
    return in_box, clear


def find_free(x, y, in_box, clear):
    """从 (x, y) 向外螺旋搜索最近的安全点"""
    for r in [1.5, 2.5, 4, 6, 8, 10, 14, 18, 24, 32]:
        for a in range(0, 360, 10):
            cx = x + r * math.cos(math.radians(a))
            cy = y + r * math.sin(math.radians(a))
            if clear(cx, cy):
                return cx, cy
    return None


def main():
    rospy.init_node('actor_escape_buildings', anonymous=False)
    boxes, obs = load_data()
    in_box, clear = make_clear_fn(boxes, obs)

    rospy.wait_for_service('/gazebo/get_model_state', timeout=60)
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pubs = {}
    for i in range(NUM_ACTOR):
        pubs[i] = rospy.Publisher('/actor_%d/cmd_motion' % i, ActorMotion,
                                  queue_size=1)
    time.sleep(1.5)

    def pos(i):
        p = get_state('actor_%d' % i, 'world').pose.position
        return p.x, p.y

    for i in range(NUM_ACTOR):
        try:
            x, y = pos(i)
        except Exception as e:
            rospy.logwarn('[escape] actor_%d 取位姿失败: %s', i, e)
            continue
        if clear(x, y):
            rospy.loginfo('[escape] actor_%d 起点 (%.1f,%.1f) 本来就空旷，跳过',
                          i, x, y)
            continue
        target = find_free(x, y, in_box, clear)
        if target is None:
            rospy.logwarn('[escape] actor_%d 找不到安全落点，放弃', i)
            continue
        tx, ty = target
        rospy.loginfo('[escape] actor_%d 起点 (%.1f,%.1f) 在建筑内 → 搬运到 '
                      '(%.1f,%.1f)', i, x, y, tx, ty)

        rate = rospy.Rate(10)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < MAX_WAIT:
            m = ActorMotion()
            m.x, m.y, m.v = tx, ty, SPEED
            pubs[i].publish(m)
            cx, cy = pos(i)
            if math.hypot(cx - tx, cy - ty) < ARRIVE:
                rospy.loginfo('[escape] actor_%d 已到达 (%.1f,%.1f)', i, cx, cy)
                break
            rate.sleep()
        else:
            cx, cy = pos(i)
            rospy.logwarn('[escape] actor_%d 超时，当前在 (%.1f,%.1f)', i, cx, cy)

    rospy.loginfo('[escape] 建筑内 actor 处理完毕')


if __name__ == '__main__':
    main()
