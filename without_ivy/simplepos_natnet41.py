#!/usr/bin/python3
import sys
sys.path.insert(0,"/home/pprz/Projects/vto-natnet/common")
from natnet41 import Rigidbody,Thread_natnet

import threading,time

acId = 888   # Rigid body tracked id
optiFreq = 120 # [100, 120, 180, 240, 360] Check that optitrack stream at least with this value to avoid duplicate captures
printFreq =120 

#------------------------------------------------------------------------------
class Thread_print(threading.Thread):
  def __init__(self,quitflag,bodies,freq):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.bodies = bodies
    self.period = 1/freq
    self.suspend = False

  def run(self):
    starttime = time.time()  
    try:
      while not self.quitflag and not (self.suspend):
        curr = self.bodies[acId]
        stamp = time.time()-starttime
        print("%.3f %d %f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f"  
          % (stamp,
             curr.valid,                                            # valid = 1, unvalid = 0
             curr.heading,                                          # rad [0 .. 2pi[ ]-2PI .. O]
             curr.position[0],curr.position[1],curr.position[2],    # meters X[-5.000,+5.000] Y[-5.000,+5.000] Z[0,+10.000]
             curr.velocity[0],curr.velocity[1],curr.velocity[2],    # meter/sec VX[-5.000,5.000] VY[-5.000,5.000] VZ[-5,5]
             curr.quat[0],curr.quat[1],curr.quat[2],curr.quat[3]))  # 
        curr.valid = False                                          # Needed if rigidbody is uncheck on optitrack/motive
        time.sleep(self.period)
    finally:
      print("Thread_print stop")
 
#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def main(bodies):
  flag = Flag()
  if len(bodies):
    try:
      threadMotion = Thread_natnet(flag,bodies,freq=optiFreq,vel_samples=1)
      threadMotion.start()
      threadPrint = Thread_print(flag,bodies,printFreq)
      threadPrint.start()
    except ValueError as msg:
      print(msg)
      exit()
      
#------------------------------------------------------------------------------
if __name__=="__main__":
  bodies = {}
  bodies[acId] = Rigidbody(acId)
#  bodies[65] = Rigidbody(65)
  main(bodies)
