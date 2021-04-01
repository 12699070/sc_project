import rospy
from geometry_msgs.msg import Twist

def talker():
    rospy.init_node('vel_publisher')
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    move = Twist()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        move.linear.x = 0.5
        # move.angular.z = 1
        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInitException:
        pass
