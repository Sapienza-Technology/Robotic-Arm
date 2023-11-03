#python ros node that uses matplotlib to plot the path of the robot
import rospy
#pose message
from nav_msgs.msg import Odometry


import numpy as np
import time
import matplotlib.pyplot as plt
import os
import message_filters
#initialize the figure
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Path')



def callback(*all_data):
    params=all_data[-1]
    #add the new point to the path
    for i in range(len(all_data)-1):
        data=all_data[i]
        topic=params["odom_topics"][i]

        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        #print only first 3 decimals
        print("topic: %s | x:%.3f y:%.3f" % (topic,x,y))
        #print(f"got data for topic {path_data['label']}:",x,y)
        #ignore if null point or point too close to the previous one
        if (x==0 and y==0) or (np.linalg.norm(np.array([x,y])-params["paths"][topic]["path"][-1])<0.02):
            continue
        params["paths"][topic]["path"] = np.append(params["paths"][topic]["path"], [[x,y]], axis=0)
        params["num_points"]+=1
        #print("topic:",path_data["label"],"\n",path_data["path"])
    
        if params["num_points"] % params["points_batch"] == 0:
            print("\nSaving picture:\n\tlocation: ",params["saving_path"])
            print()
            ax.clear()
            for topic in params["odom_topics"]:
                path=params["paths"][topic]["path"]
                ax.set_xlabel('x(m)')
                ax.set_ylabel('y(m)')
                ax.set_title('Path')
                ax.plot(path[:,0], path[:,1],label=params["paths"][topic]["label"],color=params["paths"][topic]["color"])
                ax.legend()
                #update the figure
            try:
                fig.savefig(params["saving_path"])
            except:
                print("\nError saving figure\n")
        params["rate"].sleep()




def listener():
    #initialize the node
    rospy.init_node('plot_path', anonymous=True)

    rate=rospy.Rate(5)
    odom_topics= rospy.get_param('~odom_topics')
    colors= rospy.get_param('~colours')
    #odom_topic = '/test'
    saving_path = rospy.get_param('~saving_path')
    
    print("plotting topics:",odom_topics)
    paths={}
    for i in range(len(odom_topics)):
        topic=odom_topics[i]
        col=colors[i]
        paths[topic]={"path":np.array([[0,0]]),"label":topic.replace("/",""),"color":col}

        
    
    params={"points_batch": 5* len(odom_topics), "rate": rate,"num_points": 0, "saving_path": saving_path, "paths": paths,"odom_topics":odom_topics}

    subs=[]
    for topic in odom_topics:
        #rospy.Subscriber(topic, Odometry, lambda data: callback(data,params,topic))
        sub = message_filters.Subscriber(topic, Odometry)
        subs.append(sub)


    queue_size=10
    ts = message_filters.ApproximateTimeSynchronizer(subs,queue_size,0.1)
    ts.registerCallback(callback,params)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
