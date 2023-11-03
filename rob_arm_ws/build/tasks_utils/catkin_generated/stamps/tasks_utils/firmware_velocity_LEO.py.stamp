
import rospy
from std_msgs.msg import String, Float32
#import twist message
from  geometry_msgs.msg import Twist

R=0.064
L=0.34
enc_res=356.3


MAX_V=7.0
def callback(data,pubR,pubL,rate):
    #print("received data: ", data)
    linear=data.linear.x
    angular=data.angular.z
    """ 
    in_place_vel=None
    try:
        in_place_vel=rospy.get_param("/move_base/TrajectoryPlannerROS/min_in_place_vel_theta")
    except:
        print("parameter in place velocity not found")
        pass
    if angular<0.2 and in_place_vel and angular != in_place_vel:
        linear=linear*1.5 
        angular=angular*1.7 
    """
    V_r=linear+angular*L/2
    V_l=linear-angular*L/2
    W_r=V_r/R
    W_l=V_l/R
    print("received\t  angular:  ",angular,"linear:  ",linear)
    print(f"Publishing velocity to firmware - right: {W_r} left: {W_l}")
    #publish
    print("publishing velocities to microcontroller")
    pubR.publish(W_r)
    pubL.publish(W_l)
    rate.sleep()
    

def main():
    pubR = rospy.Publisher('/destra', Float32, queue_size=10)
    pubL = rospy.Publisher('/sinistra', Float32, queue_size=10)
    
    rospy.init_node('firmware_teensy', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo firmware velocity\n\n\n")

    #subscriber pass data using lambda function
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pubR,pubL,rate))
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
