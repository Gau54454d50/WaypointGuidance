import rospy
import threading
# from dsor_msgs.msg import Measurement
from std_msgs.msg import String
from auv_msgs.msg import NavigationStatus
class Waypoint():
    def __init__(self):
        rospy.init_node('Wayptpublisher')
        self.wp_pub = rospy.Publisher("/bluerov/ctrl/waypoint",Measurement, queue_size=10)
        self.goal_sub = rospy.Subscriber("/bluerov/ctrl/goal",String,self.goal_callback)
        self.goal_flag = 0
        self.line= 0
    def goal_callback(self,msg):
        if (msg.value == 'done')&(self.goal_flag == 0) :
            self.goal_flag = 1
    def publish_wp(self) :
        while not rospy.is_shutdown():
            if self.goal_flag == 1 :
                with open("waypoint.txt", "r") as f:
                    for line in f:
                        parts=line.strip().split()
                        msg = Measurement
                        msg.value[0]=float(parts[0])
                        msg.value[1]=float(parts[1])
                        self.wp_pub.publish(msg)


while __name__ == "__main__" :
    try:
        wp = Waypoint()
        publisher_thread=threading.Thread(target=wp.publish_wp)
        publisher_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException :
        pass
