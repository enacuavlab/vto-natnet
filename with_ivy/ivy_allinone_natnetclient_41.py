#!/usr/bin/env python3
import sys
sys.path.insert(0,"../common")
from NatNetClient import NatNetClient

import argparse
from os import path, getenv
# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

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
ADDITIONAL_HELP=''

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog=ADDITIONAL_HELP,
)
parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")

#------------------------------------------------------------------------------

id_dict = dict([('51','51'),('65','65'),('69','69')]) # rigidbody_ID, aircraft_ID
freq = 10
vel_samples = 20

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
timestamp = dict([(ac_id, None) for ac_id in id_dict.keys()])
track = dict([(ac_id, deque()) for ac_id in id_dict.keys()])
rgbs = dict([(ac_id, Rigidbody(ac_id)) for ac_id in id_dict.keys()])
period = 1. / freq

#------------------------------------------------------------------------------
def store_track(ac_id, pos, t):
  if ac_id in id_dict.keys():
    track[ac_id].append((pos, t))
    if len(track[ac_id]) > vel_samples:
      track[ac_id].popleft()

def compute_velocity(ac_id):
  vel = [ 0., 0., 0. ]
  if len(track[ac_id]) >= vel_samples:
    nb = -1
    for (p2, t2) in track[ac_id]:
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


#------------------------------------------------------------------------------
def receiveRigidBodyMarkerSetList( rigid_body_data, marker_set_data, stamp ):

  for rigid_body in rigid_body_data.rigid_body_list:

    if not rigid_body.tracking_valid:
      # skip if rigid body is not valid
      continue
    i = str(rigid_body.id_num)
    if i not in id_dict.keys():
      continue
    pos = rigid_body.pos
    quat = rigid_body.rot
    store_track(i, pos, stamp)
    if timestamp[i] is None or abs(stamp - timestamp[i]) < period:
      if timestamp[i] is None:
        timestamp[i] = stamp
      continue # too early for next message
    timestamp[i] = stamp
    vel = compute_velocity(i)
    dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
    dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
    rgbs[i].quat=quat
    rgbs[i].heading=np.arctan2(dcm_1_0, dcm_0_0)
    rgbs[i].position=np.array([pos[0],pos[1],pos[2]])
    rgbs[i].velocity=np.array([vel[0],vel[1],vel[2]])
    rgbs[i].valid = True

    msg = PprzMessage("datalink", "EXTERNAL_POSE")
    msg['ac_id'] = id_dict[i]
    msg['timestamp'] = int(1000. * stamp) # Time in ms
    msg['enu_x'] = pos[0]
    msg['enu_y'] = pos[1]
    msg['enu_z'] = pos[2]
    msg['enu_xd'] = vel[0]
    msg['enu_yd'] = vel[1]
    msg['enu_zd'] = vel[2]
    msg['body_qi'] = quat[3]
    msg['body_qx'] = quat[0]
    msg['body_qy'] = quat[1]
    msg['body_qz'] = quat[2]
    ivy.send(msg)

    print(i,pos)


#------------------------------------------------------------------------------
natnet = NatNetClient()
natnet.set_server_address("192.168.1.240")
natnet.set_client_address('0.0.0.0')
natnet.set_print_level(0)  # 1 to print all frames
natnet.rigid_body_marker_set_list_listener = receiveRigidBodyMarkerSetList

args = parser.parse_args()
if args.ivy_bus is not None:
  ivy = IvyMessagesInterface("natnet2ivy", ivy_bus=args.ivy_bus)
else:
  ivy = IvyMessagesInterface("natnet2ivy")

try:
  is_running = natnet.run()
  if not is_running:
    print("Natnet error: Could not start streaming client.")
    exit(-1)
  time.sleep(1)
  if not natnet.connected():
    print("Natnet error: Fail to connect to natnet")
    exit(-1)
  while True:
    time.sleep(1)
except (KeyboardInterrupt, SystemExit):
  print("Shutting down natnet interfaces...")
  natnet.shutdown()
  ivy.shutdown()
except OSError:
  print("Natnet connection error")
  natnet.shutdown()
  ivy.stop()
  exit(-1)
