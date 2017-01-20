#!/usr/bin/python

import universal_robot
import rospy
import std_msgs.msg

from ur.msg import *
from ur.srv import *


class URNode:
  def __init__(self):
    # initialize node
    rospy.init_node('ur_node')

    # load parameters
    self.robot_ip = rospy.get_param('~robot_ip')
    self.robot_port = rospy.get_param('~robot_port')
    self.robot_data_port = rospy.get_param('~robot_data_port')
    self.rate = float(rospy.get_param('~rate'))

    # connect to robot
    self.robot = universal_robot.UR(self.robot_ip, self.robot_port, self.robot_data_port)
    self.robot.connect()

    # start state publisher
    self.topic = '~state'
    self.state_pub = rospy.Publisher(self.topic, State, queue_size=100) # todo: queue_size?
  
    # advertise services
    self.home_srv = rospy.Service('~home', Home, self.home)
    self.moveq_srv = rospy.Service('~move_q', MoveQ, self.move_q)
    self.movet_srv = rospy.Service('~move_t', MoveT, self.move_t)

  def home(self, req):
    """Moves robot to home (upright) position."""
    self.robot.move_q([0, -1.57, 0, 0, 0, 0], 0.1) # todo: hardcoded values
    return HomeResponse()

  def move_q(self, req):
    """Handles the move_q service."""
    target = req.target.data
    speed = req.speed
    self.robot.move_q(target, speed)
    return MoveQResponse()

  def move_t(self, req):
    """Handles the move_t service."""
    target = req.target.data
    speed = req.speed
    self.robot.move_t(target, speed)
    return MoveTResponse()
  
  def run(self):
    while not rospy.is_shutdown():
      # todo: add other information to the state topic
      
      # poll robot
      q = self.robot.get_q()
      pos = self.robot.get_pos()
      
      # create message
      state_msg = State()
      state_msg.header = std_msgs.msg.Header()
      state_msg.header.stamp = rospy.Time.now()
      state_msg.q.data = q
      state_msg.pos.data = pos
      self.state_pub.publish(state_msg)

      rospy.sleep(1/self.rate)


def main():
  node = URNode()
  node.run()


if __name__ == '__main__':
  main()

