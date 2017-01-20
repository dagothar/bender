"""
Defines a class for interfacinbg with UR.
"""

import socket
from ur_data import read_float, read_float_array


class UR:
  def __init__(self, robot_ip, port, data_port):
    self.robot_ip = robot_ip
    self.port = port
    self.data_port = data_port

  def connect(self):
    """
    Connects to the robot.
    Warning: THIS HAS TO BE DONE FIRST.
    """
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socket.connect((self.robot_ip, self.port))

    self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.data_socket.connect((self.robot_ip, self.data_port))
    
  def get_data_frame(self):
    """
    Reads a single data frame sent from the robot.
    """
    DATA_LENGTH = 756
    # todo: find solution to communication breakdown
    tries = 10
    while tries > 0:
      data = self.data_socket.recv(DATA_LENGTH)
      if len(data) == DATA_LENGTH:
        return data
      tries -= 1
    # todo: raise specific exception
    raise Exception

  def get_q(self):
    """
    Reads the current configuration of the robot.

    Returns:
      a list of 6 values: [q1, q2, q3, q4, q5, q6]
    """
    Q_INDEX = 252
    data = self.get_data_frame()
    q = read_float_array(data, Q_INDEX, 6)
    return q

  def get_pos(self):
    """
    Reads the current position of the robot TCP.
        
    Returns:
      a list of 6 values: [x, y, z, roll, yaw, pitch]
    """
    POS_INDEX = 588
    data = self.get_data_frame()
    pos = read_float_array(data, POS_INDEX, 6)
    return pos

  def move_q(self, target, speed):
    """
    Moves robot to a position in the configuration space.
    
    Args:
      target: a list of six joint coordinates [q1, q2, q3, q4, q5, q6] in radians
      speed: robot speed multiplier (0.0 - 1.0)
    """
    cmd = 'movej([{}], {})\n'.format(
      ', '.join([str(q) for q in target]),
      str(speed)
    )
    self.socket.send(cmd)

  def move_t(self, target, speed):
    """
    Moves robot to a position in the cartesian space.
    
    Args:
      target: a list of six tcp coordinates [x, y, z, ax, ay, az]
      speed: robot speed multiplier (0.0 - 1.0)
    """
    cmd = 'movep(p[{}], a=0.1, v={})\n'.format(
      ', '.join([str(q) for q in target]),
      str(speed)
    )
    self.socket.send(cmd)
