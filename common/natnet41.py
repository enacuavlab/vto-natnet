#!/usr/bin/env python3
import sys
from NatNetClient import NatNetClient

import time
from collections import deque
import numpy as np

#--------------------------------------------------------------------------------
#   This program gets position data from Motive Server, natnet SDK 4.1.0.0
#   and compute velocity and heading, for multiple bodies
#
#   Local Interface : 192.168.1.240
#   Transmission Type : Multicast
#   Labeled Markers : OFF
#   Unlabeled Markers : OFF
#
#   Asset Markers : ON or OFF
#   Rigid Bodies : ON
#
#   Skeletons : OFF
#   Devices : OFF
#
#   Up Axis : Z-Axis
#
#   Command Port : 1510
#   Data Port : 1511
#   Multicast Interface : 239.255.42.99
#
#------------------------------------------------------------------------------
class Rigidbody():

  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.valid = False
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)


#------------------------------------------------------------------------------
class Natnet():

  def store_track(self,ac_id, pos, t):
    if ac_id in self.rigidBodyDict.keys():
      self.track[ac_id].append((pos, t))
      if len(self.track[ac_id]) > self.vel_samples:
        self.track[ac_id].popleft()


  def compute_velocity(self,ac_id):
    vel = [ 0., 0., 0. ]
    if len(self.track[ac_id]) >= self.vel_samples:
      nb = -1
      for (p2, t2) in self.track[ac_id]:
        nb = nb + 1
        if nb == 0:
          p1 = p2
          t1 = t2
        else:
          dt = t2 - t1
          if dt < 1e-5:
            continue
          vel[0] += (p2[0] - p1[0]) / dt
          vel[1] += (p2[1] - p1[1]) / dt
          vel[2] += (p2[2] - p1[2]) / dt
          p1 = p2
          t1 = t2
      if nb > 0:
        vel[0] /= nb
        vel[1] /= nb
        vel[2] /= nb
    return vel
  
  
  def receiveRigidBodyMarkerSetList(self, rigid_body_data, marker_set_data, stamp ):
    for rigid_body in rigid_body_data.rigid_body_list:
      i = rigid_body.id_num
      if i not in self.rigidBodyDict.keys():
        continue
      if not rigid_body.tracking_valid:
        self.rigidBodyDict[i].valid = False
        continue
      pos = rigid_body.pos
      quat = rigid_body.rot
      self.store_track(i, pos, stamp)
      if self.timestamp[i] is None or abs(stamp - self.timestamp[i]) < self.period:
        if self.timestamp[i] is None:
          self.timestamp[i] = stamp
        continue # too early for next message
      self.timestamp[i] = stamp
      vel = self.compute_velocity(i)
      dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
      dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
      self.rigidBodyDict[i].quat=quat
      self.rigidBodyDict[i].heading=np.arctan2(dcm_1_0, dcm_0_0)
      self.rigidBodyDict[i].position=np.array([pos[0],pos[1],pos[2]])
      self.rigidBodyDict[i].velocity=np.array([vel[0],vel[1],vel[2]])
      self.rigidBodyDict[i].valid = True


  def __init__(self,rigidBodyDict,freq=int(20),vel_samples=int(4)):
    self.runningstate = False
    self.rigidBodyDict = rigidBodyDict
    self.period = 1.0 / freq
    self.vel_samples = vel_samples
    self.timestamp = dict([(rb, None) for rb in self.rigidBodyDict])
    self.track = dict([(rb, deque()) for rb in self.rigidBodyDict])

    self.natnet = NatNetClient()
    self.natnet.set_server_address("192.168.1.240")
    self.natnet.set_client_address('0.0.0.0')
    self.natnet.set_print_level(0)  # 1 to print all frames
    self.natnet.rigid_body_marker_set_list_listener = self.receiveRigidBodyMarkerSetList
    if self.natnet.run():
      time.sleep(1)
      if self.natnet.connected():
        self.runningstate=True
    if not self.runningstate:
        self.stop()


  def running(self):
    return (self.runningstate)


  def stop(self):
    self.natnet.shutdown()
