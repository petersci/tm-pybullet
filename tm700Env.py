import pybullet as p
import time
import math
from datetime import datetime

clid = p.connect(p.SHARED_MEMORY)
if (clid<0):
	p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,0])
#p.loadURDF("samurai.urdf")
p.loadURDF("sphere2red.urdf",[-0.1,0.2,0.63], globalScaling=0.1)
p.loadURDF("sphere_small.urdf",[0.3,0,0.63])
p.loadURDF("cube_small.urdf",[0.4,0.2,0.63])
p.loadURDF("table/table.urdf",[0,0,0], useFixedBase=True)
p.loadURDF("dinnerware/cup/cup_small.urdf",[0.3,0.3,0.6], globalScaling=2)
p.loadURDF("block.urdf",[0.3,-0.2,0.63])
p.loadURDF("random_urdfs/001/001.urdf",[0,-0.3,0.63])
p.loadURDF("random_urdfs/005/005.urdf",[0.5,0.2,0.63])
p.loadURDF("random_urdfs/007/007.urdf",[0.5,-0.2,0.63])
p.loadURDF("random_urdfs/009/009.urdf",[0.5,-0.4,0.63])
p.loadURDF("dinnerware/plate.urdf",[-0.4,-0.1,0.6])
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/sofa.urdf",[-1,1,0],[0,0,1,1], useFixedBase=True)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/tv.urdf",[0,-1,0.6],[0,0,1,0],useFixedBase=True)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/sideboard.urdf",[-0.7,-1.3,0],[0,0,1,1],useFixedBase=True)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/arm_chair.urdf",[-1,0,0],useFixedBase=True)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/closet.urdf",[-1.7,1.2,0],useFixedBase=True)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/stool.urdf",[0.7,0.7,0],useFixedBase=True, globalScaling=1.5)
p.loadURDF("robocup-code/robocup_stacks/simulation/robocup_worlds/objects/lamp.urdf",[0.3,-1.2,0],useFixedBase=True, globalScaling=1.5)

kukaId = p.loadURDF("tm_description/urdf/tm700simple.urdf",[0,0,0.6],[0,0,0,1], useFixedBase=True)
#p.resetBasePositionAndOrientation(kukaId,[0,0,0],[0,0,0,1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints!=7):
	exit()

p.setGravity(0,0,-10)
camInfo = p.getDebugVisualizerCamera()
camEyePos = [0,0,2]
camTarPos = [0,0,0]
camUpVec = [0,-1,0]
viewMat = p.computeViewMatrix(camEyePos,camTarPos,camUpVec)
projMatrix = camInfo[3]
p.getCameraImage(320,200, viewMatrix=viewMat, projectionMatrix=projMatrix, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
segLinkIndex=1
verbose = 0

while (1):
	keys = p.getKeyboardEvents()
	#for k in keys:
	#	print("key=",k,"state=", keys[k])
	if ord('1') in keys:
		state = keys[ord('1')]
		if (state & p.KEY_WAS_RELEASED):
			verbose = 1-verbose
	if ord('s') in keys:
		state = keys[ord('s')]
		if (state & p.KEY_WAS_RELEASED):
			segLinkIndex = 1-segLinkIndex
			#print("segLinkIndex=",segLinkIndex)
	flags = 0
	if (segLinkIndex):
		flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
		
	img = p.getCameraImage(320,200, viewMatrix=viewMat, projectionMatrix=projMatrix, flags=flags)
	#print(img[0],img[1])
	seg=img[4]
	if (verbose):
		for pixel in seg:
			if (pixel>=0):
				obUid = pixel & ((1<<24)-1)
				linkIndex = (pixel >> 24)-1
				print("obUid=",obUid,"linkIndex=",linkIndex)
	
	
	p.stepSimulation()

'''	
#p.getCameraImage(320,200,flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
#segLinkIndex=1
#verbose = 0


#lower limits for null space
ll=[-.967,-2	,-2.96,0.19,-2.96,-2.09,-3.05]
#upper limits for null space
ul=[.967,2	,2.96,2.29,2.96,2.09,3.05]
#joint ranges for null space
jr=[5.8,4,5.8,4,5.8,4,6]
#restposes for null space
rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
#joint damping coefficents
jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]

for i in range (numJoints):
	p.resetJointState(kukaId,i,rp[i])

p.setGravity(0,0,-1)
t=0.
prevPose=[0,0,0]
prevPose1=[0,0,0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
	
while 1:
	if (useRealTimeSimulation):
		dt = datetime.now()
		t = (dt.second/60.)*2.*math.pi
	else:
		t=t+0.001
	
	if (useSimulation and useRealTimeSimulation==0):
		p.stepSimulation()

'''
