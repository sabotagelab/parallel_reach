import rospy
import yaml

class Reachability:
    def __init__(self):
        rospy.init_node('reachability_node', anonymouse=True)

        #subscribe to mpc input predictions
        #subscribe to current state 

    def start(self):
        rospy.spin()

    
if __name__ == "__main__":
    node = Reachability("settings.yaml")
    node.start()