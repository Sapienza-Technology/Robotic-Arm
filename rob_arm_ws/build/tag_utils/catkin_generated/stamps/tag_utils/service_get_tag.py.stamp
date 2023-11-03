#ROS node that implements a service, it receives the tag id and find in a file the position of the tag

import rospy
import sys
import os
import json
#import the custom service in the srv foilder of this package: GetTag.srv
from tag_utils.srv import GetTag, GetTagResponse

"""
Call this service using the following code

import rospy
from tag_utils.srv import *

rospy.wait_for_service('get_tag')
try:
    get_tag = rospy.ServiceProxy('get_tag', GetTag)
    response = get_tag(0) #list of 3 elements (x,y,z)
    print(response)
except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
"""

def handle_get_tag(req, params):
    if params["tag_pos_dict"] == None:
        tag_pos = [0,0,0]
        return GetTagResponse(tag_pos)
    #get the tag id
    tag_id=int(req.id)
    print("TAG_SRV: requested tag id: ", tag_id)
    #get the tag position
    tag_pos=params["tag_pos_dict"].get(str(tag_id))
    if tag_pos is None:
        tag_pos=[0,0,0]          
    print("TAG_SRV: sending back tag position: ", tag_pos)
    return GetTagResponse(tag_pos)
    

def main():
    rospy.init_node('service_get_tag', anonymous=True)
    #read the file
    path=os.path.dirname(os.path.realpath(__file__))
    path=os.path.dirname(path)
    path=os.path.dirname(path)
    path=path+"/files/tag_pos.json"

    #read file
    try:
        with open(path, 'r') as fp:
            tag_pos_dict = json.load(fp)
    except:
        tag_pos_dict = None
    
    params={
        "tag_pos_dict":tag_pos_dict
    }
    check_service = rospy.Service('get_tag', GetTag, lambda req: handle_get_tag(req, params))
    rospy.loginfo("Tag service ready")
    rospy.spin()

if __name__ == "__main__":
    main()