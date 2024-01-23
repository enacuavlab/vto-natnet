#!/usr/bin/python3
import sys
sys.path.insert(0,"../common")
from natnet41 import Rigidbody,Natnet

import threading,time

acId = 51   # Rigid body tracked id
optiFreq = 120 # [100, 120, 180, 240, 360] Check that optitrack stream at least with this value to avoid duplicate captures
printFreq =120 

#------------------------------------------------------------------------------
class Thread_print(threading.Thread):
  def __init__(self,bodies,freq):
    threading.Thread.__init__(self)
    self.quitflag = False
    self.bodies = bodies
    self.period = 1/freq

  def run(self):
    starttime = time.time()  
    try:
      while not self.quitflag:
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
 
  def stop(self):
    self.quitflag = True


#------------------------------------------------------------------------------
def main(bodies):
  if len(bodies):
    try:
      motion = Natnet(bodies,freq=optiFreq,vel_samples=1)
      if not motion.running(): 
       exit(-1)

      threadPrint = Thread_print(bodies,printFreq)
      threadPrint.start()

      while True:
        time.sleep(1)

    except (KeyboardInterrupt, SystemExit):
      threadPrint.stop()
      motion.stop()

    except ValueError as msg:
      print(msg)
      exit()
      
#------------------------------------------------------------------------------
if __name__=="__main__":
  bodies = {}
  bodies[acId] = Rigidbody(acId)
  main(bodies)
