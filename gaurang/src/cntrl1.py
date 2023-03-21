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
        self.depth_pub=rospy.Publisher("/bluerov/ref/depth",Float64,queue_size=10)
        self.altitude_pub=rospy.Publisher("/bluerov/ref/altitude",Float64,queue_size=10)
        self.tolerance=2
        #self.warn_alt = 2
        #self.danger_alt = 1
        self.dist=None
        #self.depthalt_sub=rospy.Subscriber("/bluerov/measurement/position",Measurement,self.depthalt_callback)
        self.pos_sub=rospy.Subscriber("/bluerov/nav/filter/state_dr", NavigationStatus, self.dvl_callback)
        self.timer=rospy.Timer(rospy.Duration(0.1),self.timer_callback)
        self.wp = []
        self.index = int(0)
        #self.altitude = None 
        #self.depth = None

        with open("waypoint1.txt","r") as f:
            for coordinates in f:
                wp=coordinates.strip().split()
                self.wp.append([float(wp[0]),float(wp[1]),int(wp[2]),int(wp[3])])
        if len(self.wp) == 0:
            print("error reading file")
            exit()

    def dvl_callback(self,msg):
        self.x=msg.position.north
        self.y=msg.position.east
        self.x=math.floor(self.x)
        self.y=math.floor(self.y)
        print(self.x,self.y,"actual_position")
        self.x_des=self.wp[self.index][0]
        self.y_des=self.wp[self.index][1]
        print(self.x_des,self.y_des,"destination")
        psi_r = math.atan2(self.y_des-self.y,self.x_des-self.x)
        self.psi_d= math.degrees(psi_r)
        self.psi_d=math.floor(self.psi_d)
        self.yaw_pub.publish(self.psi_d)
        self.dist=math.sqrt((self.y_des-self.y)**2+(self.x_des-self.x)**2)
        print(self.dist,"distance")
        if self.index == len(self.wp):
            rospy.shutdown()
            input()
            exit()
        return 0 
    
    def timer_callback(self,event):
        if self.index == len(self.wp):
            rospy.signal_shutdown("no points remaining")
            input()
            exit()
        if self.wp[self.index][2] == 0:
            self.depth_pub.publish(self.wp[self.index][3])
            print(f"depth={+self.wp[self.index][3]}")
        if self.wp[self.index][2] == 1:
            self.altitude_pub.publish(self.wp[self.index][3])
            print(f"altitude={+self.wp[self.index][3]}")
        if self.dist == None:
            return 0
        if self.dist>=self.tolerance:
            self.surge_pub.publish(0.5)
        else:
            print("Waypoint achieved! Moving to next Waypoint")
            self.dist = None
            self.surge_pub.publish(0.0)
            self.index+=1
            if self.index == len(self.wp):
                exit()
                input("end of file")
                input("stopped")
                exit()
        
    """ def depthalt_callback(self,msg):
        if msg.header.frame_id=="bluerov_altimeter":
            if math.floor(msg.value[0]) <= self.warn_alt:
                print("Warning approaching surface")
            if math.floor(msg.value[0]) <= self.danger_alt:
                if self.wp[self.index][2] == 0 :
                    self.wp[self.index][3] = self.wp[self.index][3] - self.warn_alt
                    print("reducing depth by 1")
                if self.wp[self.index][2]  ==1 :
                    self.wp[self.index][3] = self.wp[self.index][3] + self.warn_alt 
                    print("reducing altitude by 1")
            if math.floor(msg.value[0]) == 0:
                if self.wp[self.index][2] == 0 :
                    self.wp[self.index][3] = 0 
                    print("reducing depth by 1")
                if self.wp[self.index][2]  ==1 :
                    self.wp[self.index][3] = 10
                    print("reducing altitude by 1")"""

 
            


if __name__=="__main__":
    rospy.init_node("cntrl")
    rate =rospy.Rate(20)
    c=Cntrl()
    subthread1= threading.Thread(target= c.dvl_callback)
    #subthread2= threading.Thread(target = c.depthalt_callback)
    while not rospy.is_shutdown():
        rospy.spin()