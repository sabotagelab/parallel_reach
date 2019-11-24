#node to display reachsets as filled planes in rviz
# subscribes to hylaa output topic
# publishes RVIZ triangle list
# configurable in hylaa_viz.yaml

import rospy

class Viz_Lineq:
    def __init__(self):
        rospy.init_node("hylaa_viz")


        reach_sub_topic = rospy.get_param("/hylaa_node/reach_pub_topic", "reach_pub")
        reach = rospy.Subscriber(reach_sub_topic)

if __name__ == "__main__":


