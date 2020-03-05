#!/usr/bin/python
#node to display reachsets as filled planes in rviz
# subscribes to hylaa output topic
# publishes RVIZ triangle list
# configurable in hylaa_viz.yaml

import rospy
from osuf1_common.msg import ReachSets
import tripy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from osuf1_common.msg import ReachSets
from itertools import repeat

#node to publish reachability visualization messages for viewing in rviz

class Hylaa_Viz_Node:
    def __init__(self):
        rospy.init_node("hylaa_viz")


        reach_sub_topic = rospy.get_param("/zono_node/reach_pub_topic", "reach_pub")
        self.reachPub = rospy.Subscriber(reach_sub_topic, ReachSets, self.makeViz) 
        sets_outline_topic = rospy.get_param("/hylaa_viz/outline_topic", "hylaa_viz_outline")
        sets_tris_topic = rospy.get_param("/hylaa_viz/tris_topic", "hylaa_viz_tris")

        self.outlinePub = rospy.Publisher(sets_outline_topic, MarkerArray, queue_size=1)
        self.trisPub = rospy.Publisher(sets_tris_topic, Marker, queue_size=1)

        self.tri_marker_id = 44
        self.outline_marker_id = 555 # + n

        self.colors = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 190), (0, 128, 128), (230, 190, 255), (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195), (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128)]
        self.colors = [ColorRGBA(c[0], c[1], c[2], 1) for c in self.colors]

    def start(self):
        rospy.spin()
    
    def makeViz(self, reachSets):
        pointSets = [[tuple(p.p) for p in rs.set] for rs in reachSets.sets]
        triangleSets = [tripy.earclip(ps) for ps in pointSets]


        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = reachSets.header.frame_id

        origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

        lineMarkers = MarkerArray()
        lineMarkerArray = []
        for ii in range(len(pointSets)):
            m = Marker()
            m.header = header
            m.id = self.outline_marker_id + ii
            m.action = 0
            m.pose = origin
            m.type = 4 #LINE_STRIP
            m.color = self.colors[ii%len(self.colors)]
            m.points = [Point(p[0], p[1], 0) for p in pointSets[ii]]
            m.scale = Vector3(.05, 0, 0)
            lineMarkerArray.append(m)
        
        lineMarkers.markers = lineMarkerArray 

        self.outlinePub.publish(lineMarkers)


        triPoints = [ xy for tri in triangleSets for xy in tri]

        triMarker = Marker()
        triMarker.header = header
        triMarker.id = self.tri_marker_id
        triMarker.type = 11 #TRIANGLE_LIST
        triMarker.action = 0
        triMarker.pose = origin
        triMarker.color = ColorRGBA(1, 1, 1, 1)
        triMarker.scale = Vector3(1, 1, 1)
        triMarker.points = [Point(p[0], p[1], 0) for tri in triPoints for p in tri]

        #expand color array to cover all verts for all tris in each set with same color
        triFrequency = [ len(ps) for ps in pointSets ]
        triColors = [ self.colors[ii%len(self.colors)] for ii in range(len(pointSets))]
        triMarker.colors = [c for cidx in range(len(triColors)) for c in repeat(triColors[cidx], triFrequency[cidx])]

        self.trisPub.publish(triMarker)

    #returns new rgba color tuple with alpha added from rgb tupe and a val
    def plusAlpha(self, ct, a):
        return ColorRGBA(ct[0], ct[1], ct[2], a)


if __name__ == "__main__":
    node = Hylaa_Viz_Node()
    node.start()

