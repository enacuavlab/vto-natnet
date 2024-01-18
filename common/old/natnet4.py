#!/usr/bin/python3

import numpy as np
import struct
import socket
import select
import threading
import time
import subprocess

from collections import deque

#--------------------------------------------------------------------------------
# From Optitrack Natnet rigidbodies live positions
#   This programs inspired  from MoCapData.py natnet SDK 4.0 (windows package)
#   This program gets data from Motive Server v3, with following settings:
#   Local Interface : 192.168.1.231
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

Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )

Optitrack = '192.168.1.231'

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
class Thread_natnet(threading.Thread):

  def __init__(self,quitflag,rigidBodyDict,freq=int(20),vel_samples=int(4)):
    threading.Thread.__init__(self)

    p = subprocess.Popen(["ping", "-q", "-c", "1", Optitrack], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if p.wait() != 0:
      raise ValueError('Optitrack not in this network')

    self.quitflag = quitflag
    self.rigidBodyDict = rigidBodyDict
    self.period = 1.0 / freq
    self.vel_samples = vel_samples
    self.timestamp = dict([(rb, None) for rb in self.rigidBodyDict])
    self.track = dict([(rb, deque()) for rb in self.rigidBodyDict])

    self.data_sock = socket.socket( socket.AF_INET,socket.SOCK_DGRAM,0)
    self.data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.data_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton("239.255.42.99") + socket.inet_aton("0.0.0.0"))
    try:
      self.data_sock.bind( ("0.0.0.0", 1511) )
    except socket.error as msg:
      print("ERROR: data socket error occurred:\n%s" %msg)
      print("  Check Motive/Server mode requested mode agreement.  You requested Multicast ")



  def run(self):
    data=bytearray(0)
    # 64k buffer size
    recv_buffer_size=64*1024
    store_time = 0
    try:
      while not self.quitflag:
        data, addr = self.data_sock.recvfrom( recv_buffer_size )
        if len( data ) > 0 :
  
          current_time = time.time()
          if store_time == 0: store_time= current_time
          elif current_time - store_time > self.period:
            store_time = current_time
  
          message_id = int.from_bytes( data[0:2], byteorder='little' )
          packet_size = int.from_bytes( data[2:4], byteorder='little' )
          if message_id == 7 : # NAT_FRAMEOFDATA :
            offset = 4
            offset += 4
            marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
      
            for i in range( 0, marker_set_count ): #  if Asset Markers : ON
              model_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
              offset += len( model_name ) + 1
              marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
              offset += 4
              for j in range( 0, marker_count ):
                floatLst = Vector3.unpack( data[offset:offset+12] )
                offset += 12
      
            offset += 4
            rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            for i in range( 0, rigid_body_count ): # if Rigid Bodies : ON
              ac_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
              offset += 4
              pos = Vector3.unpack( data[offset:offset+12] )
              offset += 12
              quat = Quaternion.unpack( data[offset:offset+16] )
              offset += 16
              marker_error, = FloatValue.unpack( data[offset:offset+4] )
              offset += 4
              param, = struct.unpack( 'h', data[offset:offset+2] )
              tracking_valid = ( param & 0x01 ) != 0
              offset += 2
  
              if ac_id in self.rigidBodyDict:
  
                if not tracking_valid: 
                  self.rigidBodyDict[ac_id].valid = False
                  continue
  
                stamp = time.time()
                self.track[ac_id].append((pos, stamp))
                if len(self.track[ac_id]) > self.vel_samples:
                  self.track[ac_id].popleft()
  
                if self.timestamp[ac_id] is None or abs(stamp - self.timestamp[ac_id]) < self.period:
                  if self.timestamp[ac_id] is None: self.timestamp[ac_id] = stamp; continue # too early for next message
                self.timestamp[ac_id] = stamp
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
                      if dt < 1e-5: continue
                      vel[0] += (p2[0] - p1[0]) / dt
                      vel[1] += (p2[1] - p1[1]) / dt
                      vel[2] += (p2[2] - p1[2]) / dt
                      p1 = p2
                      t1 = t2
                  if nb > 0:
                    vel[0] /= nb
                    vel[1] /= nb
                    vel[2] /= nb
                dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
                dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
                self.rigidBodyDict[ac_id].quat=quat
                self.rigidBodyDict[ac_id].heading=np.arctan2(dcm_1_0, dcm_0_0)
                self.rigidBodyDict[ac_id].position=np.array([pos[0],pos[1],pos[2]])
                self.rigidBodyDict[ac_id].velocity=np.array([vel[0],vel[1],vel[2]])
                self.rigidBodyDict[ac_id].valid = True
  

    finally:
      self.data_sock.close()
      print("Thread_natnet stop")
