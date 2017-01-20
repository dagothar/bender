"""
Defines functions for translating data sent to and received from the robot.
"""

import struct


def read_float(data, index):
  """
  Reads a float value from the data frame.

  Args:
    data - data frame
    index - the index of the first byte of the float value
    
  Returns:
    float value stored at specified index
  """
  tmp = data[index:index+8]
  f = (struct.unpack('d', tmp[::-1]))[0]
  return f


def read_float_array(data, index, n):
  """
  Reads an array float value from the data frame.

  Args:
    data - data frame
    index - the index of the first byte of the float array
    n - number of float values in the array
  Returns:
    float value array stored at specified index
  """
  fs = []
  for i in range(0, n):
    fs.append(read_float(data, index+i*8))
  return fs
