import rospy
import math

import numpy as np
import tf



class Node:

    def __init__(self):
        rospy.init_node("object_detection")

        
    
    #we will assume that the depth more consistent than the tag
    #under this assumption, we will use the depth data to trigger a callback
    #this is logical since then it is more likely that the variation will be normal
    #^^that logic is probably bs

    #here we recieve the image depth data
    def recieveDepthData(self, depth):

    #here we recieve the boundoing boxes for the april tags
    def recieveTagData(self, tag):
        self.currentTagData = tag#TODO determine correct property
    
        

        
