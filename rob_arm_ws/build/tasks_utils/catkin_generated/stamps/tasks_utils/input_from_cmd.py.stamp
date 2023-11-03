#simple ros node that takes input from cmd (from launch file) and print them
#Launch files i launched using roslaunch pkg_name launch_file.launch tags:="1,2,3,4"
#I want tot ake the tag parameter and use it in the node
import rospy
import sys
def main():
    rospy.init_node('input_from_cmd', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo input from cmd\n\n\n")
    print("tags: ",sys.argv[1])
    
    rospy.spin()

if __name__ == '__main__':
    main()