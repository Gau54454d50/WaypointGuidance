import rospy
import math
from auv_msgs.msg import NavigationStatus
from dsor_msgs.msg import Measurement
from std_msgs.msg import Float64
import threading

class Cntrl():
    def __init__(self):
        self.yaw_pub=rospy.Publisher("/bluerov/ref/yaw", Float64, queue_size=10)
        self.surge_pub=rospy.Publisher("/bluerov/ref/surge", Float64, queue_size=10)
        self.tolerance=2
        self.pos_sub=rospy.Subscriber("/bluerov/measurement/position", Measurement, self.utm_callback)
        self.timer=rospy.Timer(rospy.Duration(0.1),self.timer_callback)
        self.wp = []
        self.index = int(0)
        with open("waypoint.txt","r") as f:
            for coordinates in f:
                wp=coordinates.strip().split()
                self.wp.append([float(wp[0]),float(wp[1])])
        if len(self.wp) == 0:
            print("error reading file")
            exit()

    def utm_callback(self,msg):
        if msg.header.frame_id=="bluerov_gnss":
            self.x=msg.value[0]
            self.y=msg.value[1]
            self.x=math.floor(self.x)
            self.y=math.floor(self.y)
            print(self.x,self.y)
            self.x_des=self.wp[self.index][0]
            self.y_des=self.wp[self.index][1]
            print(self.x_des,self.y_des)
            psi_r = math.atan2(self.y_des-self.y,self.x_des-self.x)
            self.psi_d= math.degrees(psi_r)
            self.psi_d=math.floor(self.psi_d)
            self.yaw_pub.publish(self.psi_d)
            self.dist=math.sqrt((self.y_des-self.y)**2+(self.x_des-self.x)**2)
            print(self.dist)
            if self.index == len(self.wp):
                input()
                exit()
        return 0
    
    def timer_callback(event):
        if self.dist>=self.tolerance:
            self.surge_pub.publish(1.0)

        else:
            print("Waypoint achieved! Moving to next Waypoint")
            self.surge_pub.publish(0.0)
            self.index+=1


if __name__=="__main__":
    rospy.init_node("cntrl")
    rate =rospy.Rate(20)
    c=Cntrl()
    subthread = threading.Thread(target= c.utm_callback)
    while not rospy.is_shutdown():
        rospy.spin()