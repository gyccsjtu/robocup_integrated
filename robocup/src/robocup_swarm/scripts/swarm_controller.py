#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped


class SwarmController:

    def __init__(self):

        rospy.init_node("swarm_controller")


        # 六架无人机
        self.uavs = [
            "uav_1",
            "uav_2",
            "uav_3",
            "uav_4",
            "uav_5",
            "uav_6"
        ]


        self.pose = {}

        self.cmd_pub = {}


        # 偏移队形
        self.offset = {

            "uav_2": [2,0,0],
            "uav_3": [0,2,0],
            "uav_4": [-2,0,0],
            "uav_5": [0,-2,0],
            "uav_6": [2,2,0]

        }


        for uav in self.uavs:


            rospy.Subscriber(
                "/"+uav+"/mavros/local_position/pose",
                PoseStamped,
                self.pose_callback,
                callback_args=uav
            )


            self.cmd_pub[uav] = rospy.Publisher(

                "/"+uav+"/mavros/setpoint_velocity/cmd_vel",
                TwistStamped,
                queue_size=10
            )


    def pose_callback(self,msg,uav):

        self.pose[uav]=msg.pose.position



    def control(self):


        if "uav_1" not in self.pose:
            return


        leader=self.pose["uav_1"]


        for uav in self.uavs[1:]:


            if uav not in self.pose:
                continue


            target_x = leader.x + self.offset[uav][0]
            target_y = leader.y + self.offset[uav][1]
            target_z = leader.z


            error_x = target_x-self.pose[uav].x
            error_y = target_y-self.pose[uav].y
            error_z = target_z-self.pose[uav].z


            cmd=TwistStamped()


            k=0.5


            cmd.twist.linear.x=k*error_x
            cmd.twist.linear.y=k*error_y
            cmd.twist.linear.z=k*error_z


            self.cmd_pub[uav].publish(cmd)



    def run(self):

        rate=rospy.Rate(20)

        while not rospy.is_shutdown():

            self.control()

            rate.sleep()



if __name__=="__main__":

    node=SwarmController()

    node.run()
