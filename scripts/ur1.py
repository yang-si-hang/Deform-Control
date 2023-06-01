import urx
import math3d as m3d
import numpy as np

rob = urx.Robot("192.168.100.102")
rob.set_tcp((0, 0, 0, 0, 0, 0)) 
rob.set_payload(2, (0, 0, 0.1))
# sleep(0.2)  #leave some time to robot to process the setup commands
print ("Current tool pose is: ",  rob.getl())
trans = rob.get_pose()
print(trans.array)
tool=np.array([[0.0],[0.0],[0.08], [1]])
print(trans.array @ tool)


# trans.orient.rotate_xb(+np.pi/6)

# rob.set_pose(trans, acc=0.5, vel=0.2)
# print (trans)
# rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
# rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
rob.close()