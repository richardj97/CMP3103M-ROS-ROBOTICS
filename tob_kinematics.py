import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wheel_radius = .1
robot_radius = .1
    
    
        # computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
        c_l = wheel_radius * w_l
        c_r = wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_r - c_l) / (2 * robot_radius)
        return (v, a)
    
    
    # computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
        c_l = v - (robot_radius * a)
        c_r = v + (robot_radius * a)
        w_l = c_l / wheel_radius
        w_r = c_r / wheel_radius
        return (w_l, w_r)
    
    
    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(self, t):
        return self.inverse_kinematics(t.linear.x, t.angular.z)
        
        
class left_wheel():

    def __init__(self):
        print('Starting...')
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.float32_sub = rospy.Subscriber("/wheel_vel_left",
                                              Float32, self.callback)
                                           
        
    def callback(self, data):
        print(data.data)
        (v,a) = forward_kinematics (data.data, 0)
        
        t = Twist()
        t.linear.x = v
        t.angular.z = a
        self.pub.publish(t)
        
        
                                        


if __name__ == "__main__":
    
    left_wheel()    
    rospy.init_node('left_wheel', anonymous=True)
    rospy.spin()    
    
#    (w_l, w_r) = inverse_kinematics(0.0, 1.0)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)

#    (v, a) = forward_kinematics(w_l, w_r)
#    print "v = %f,\ta = %f" % (v, a)
    
#    t = Twist()
#    t.linear.x = 0.3
#    t.angular.z = 0.8

#    (w_l, w_r) = inverse_kinematics_from_twist(t)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)