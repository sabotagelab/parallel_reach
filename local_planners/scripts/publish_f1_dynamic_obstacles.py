#!/usr/bin/env python

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker,MarkerArray

class DynamicObstacles:
  def __init__(self):
    rospy.init_node("test_obstacle_msg")
    self.obstacle_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    self.marker_pub = rospy.Publisher('dynamic_obstacle_markers', MarkerArray, queue_size=1)
    self.obstacle_msg = ObstacleArrayMsg()
    self.obs_array_markers = MarkerArray()
    self.range_x=[]
    self.range_y=[]
    self.pose_x=[]
    self.pose_y=[]

  def publish_obstacle_msg(self):
    self.obstacle_msg.header.stamp = rospy.Time.now()
    self.obstacle_msg.header.frame_id = "map"
    # Add point obstacle
    self.obstacle_msg.obstacles.append(self.add_obstacle(1,23.79,19.09,1.638,-1.1472,14.79,-10.358))
    self.obstacle_msg.obstacles.append(self.add_obstacle(2, 20.06,16.6066, 2.033, 2.205, -31.759, -34.435))
    self.obstacle_msg.obstacles.append(self.add_obstacle(3, 4.137,0.063, 1.388, 1.439, 11.09, 11.501))
    self.obs_array_markers = self.create_obstacle_markers()

    r = rospy.Rate(20) # 10hz
    t = 0.0
    while not rospy.is_shutdown():
      self.move_obstacle(t)
      self.marker_pub.publish(self.obs_array_markers)
      self.obstacle_pub.publish(self.obstacle_msg)
      t = t + 0.05
      r.sleep()

  def add_obstacle(self,id,posx,posy,vel_x,vel_y,range_x,range_y):
    self.pose_x.append(posx)
    self.pose_y.append(posy)
    self.range_x.append(range_x)
    self.range_y.append(range_y)
    samp_obstacle= ObstacleMsg()
    samp_obstacle.id = id
    samp_obstacle.radius= 0.35
    samp_obstacle.polygon.points = [Point32()]
    samp_obstacle.polygon.points[0].x = posx
    samp_obstacle.polygon.points[0].y = posy
    samp_obstacle.polygon.points[0].z = 0

    yaw = math.atan2(vel_y, vel_x)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    samp_obstacle.orientation = Quaternion(*q)

    samp_obstacle.velocities.twist.linear.x = vel_x
    samp_obstacle.velocities.twist.linear.y = vel_y
    samp_obstacle.velocities.twist.linear.z = 0
    samp_obstacle.velocities.twist.angular.x = 0
    samp_obstacle.velocities.twist.angular.y = 0
    samp_obstacle.velocities.twist.angular.z = 0
    return samp_obstacle

  def move_obstacle(self,t):
    for idx,obstacle in enumerate(self.obstacle_msg.obstacles):
      obstacle.polygon.points[0].x = self.pose_x[idx] + (obstacle.velocities.twist.linear.x * t)%self.range_x[idx]
      obstacle.polygon.points[0].y = self.pose_y[idx] +(obstacle.velocities.twist.linear.y * t) % self.range_y[idx]
      self.obs_array_markers.markers[idx].pose.position.x = obstacle.polygon.points[0].x
      self.obs_array_markers.markers[idx].pose.position.y = obstacle.polygon.points[0].y

  def create_obstacle_markers(self):
    obs_array = MarkerArray()
    for i in range(len(self.obstacle_msg.obstacles)):
      marker = Marker()
      marker.id=i+1
      marker.header.frame_id = "/map"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.3
      marker.scale.y = 0.3
      marker.scale.z = 0.3
      marker.color.a = 1.0
      marker.color.r = 0.0
      marker.color.g = 0.0
      marker.color.b = 1.0
      marker.pose.orientation.w = 1.0
      marker.pose.position.x = self.obstacle_msg.obstacles[i].polygon.points[0].x
      marker.pose.position.y = self.obstacle_msg.obstacles[i].polygon.points[0].y
      marker.pose.position.z = 0
      obs_array.markers.append(marker)
    return obs_array

if __name__ == '__main__':
  dynamic_obstacles = DynamicObstacles()
  try:
    dynamic_obstacles.publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

