from openravepy import *
from numpy import *
import time
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('../../robots/er4u.env.xml') # load a simple scene
time.sleep(0.5) #wait for things to intialize
robot = env.GetRobots()[0] # get the first robot

RaveSetDebugLevel(DebugLevel.Verbose) # set output level to debug
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
manip = robot.GetActiveManipulator()
 
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()

#with env: # lock the environment since robot will be used
raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\\n"+repr(robot.GetDOFValues()))
robot.SetDOFValues([0.5,-0.5,0.5,0.5,0.5,0.5,0.5],[0,1,2,3,4,5,6])
env.UpdatePublishedBodies()
time.sleep(1)
T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
TEnd = manip.GetEndEffectorTransform()

raveLogInfo("The transformation of link 1 is:\n"+repr(T))
raveLogInfo("\nThe transformation of end effector is:\n"+repr(TEnd))
raveLogInfo("Moving on...")

#Now try using inverse kinematics
target=env.GetKinBody("cube")
destination_point=array([0.01,0.05,0.8])#
raveLogInfo("\nThe target Transform is "+repr(destination_point))
#direction = random.rand(3)-0.5
direction = [0,0,1]#Give it an approach direction, e.g. here Z axis
direction /= linalg.norm(direction)#Normalize the direction
destination_point = target.GetTransform()[0:3,3]+[0,0,0.04] #Extract only the translation vector from T
handle=env.drawlinestrip(array([destination_point,destination_point+0.1*direction]),10)#Need to assign a handle to ensure that the strip is visible
solutions = ikmodel.manip.FindIKSolutions(IkParameterization(Ray(destination_point,direction),IkParameterization.Type.TranslationDirection5D),IkFilterOptions.CheckEnvCollisions)
raveLogInfo("\nThe solution for the ik Parameters is ")
print solutions
if solutions is not None and len(solutions)>0:
    print solutions
    traj=manipprob.MoveManipulator(solutions[0],execute=False,outputtrajobj=True)
else:
    traj=None
    RaveLogError("Couldn't find a solution!!!")
    
#Trying out grasping now    
gmodel = databases.grasping.GraspingModel(robot,target=target)
if not gmodel.load():
    print '\n\n\ngenerating grasping model (one time computation)\n\n'
    time.sleep(2)
    gmodel.init(friction=0.4,avoidlinks=[])
    gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04,normalanglerange=0))
    gmodel.save()

returnnum = 5

print 'computing first %d valid grasps'%returnnum
validgrasps,validindices = gmodel.computeValidGrasps(returnnum=returnnum)
for validgrasp in validgrasps:
    gmodel.showgrasp(validgrasp,useik=True,collisionfree=True,delay=5)
    time.sleep(5)
print validgrasps
    
    
raveLogInfo("And on...")
if traj is not None:
    robot.GetController().SetPath(traj)
    robot.WaitForController(0) # wait
raveLogInfo("Ta!")
time.sleep(3)
