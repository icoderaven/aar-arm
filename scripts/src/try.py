from openravepy import *
from numpy import *
import time
import openravepy
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/kshaurya/aar-arm/robots/er4u.env.xml') # load a simple scene
time.sleep(0.5) #wait for things to intialize
robot = env.GetRobots()[0] # get the first robot

RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
manip = robot.GetActiveManipulator()

 
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()

with env: # lock the environment since robot will be used
    raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\\n"+repr(robot.GetDOFValues()))
    robot.SetDOFValues([0.1,-0.6,0.3,0.7,0.5],[0,1,2,3,4])
    env.UpdatePublishedBodies()
    time.sleep(1)
    T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
    TEnd = manip.GetEndEffectorTransform()
    
    robot.GetDOFValues
    raveLogInfo("The transformation of link 1 is:\n"+repr(T))
    raveLogInfo("\nThe transformation of end effector is:\n"+repr(TEnd))
    raveLogInfo("Moving on...")
    # call motion planner with goal joint angles
    raveLogInfo(str(len(robot.GetActiveManipulator().GetArmIndices())))
    traj = manipprob.MoveManipulator(goal=[0,0,0.2,0.5,0.6],execute=False, outputtrajobj=True)
    #Now try using inverse kinematics
    target=env.GetKinBody("cube")
    destination_point=array([0.03,0.08,0.8])#
    #destination_point = target.GetTransform()[0:3,3] #Extract only the translation vector from T
#    direction=array([0.0,0.0,0.0])  #Give it an approach direction, e.g. here Z axis
#    direction /= linalg.norm(direction) #Normalize the direction
    raveLogInfo("\nThe target Transform is "+repr(destination_point))
    
    while True:
        #direction = random.rand(3)-0.5
        direction = destination_point-TEnd[0:3,3]
        direction /= linalg.norm(direction)
        #Need to assign a handle to ensure that the strip is visible
        handle=env.drawlinestrip(array([destination_point,destination_point+0.1*direction]),10)
        solutions = ikmodel.manip.FindIKSolutions(IkParameterization(Ray(destination_point,direction),IkParameterization.Type.TranslationDirection5D),IkFilterOptions.CheckEnvCollisions)
        if solutions is not None and len(solutions) > 0: # if found, then break
            break
#    ikparam=IkParameterization(Ray(destination_point,direction), IkParameterization.Type.TranslationDirection5D)
#    solution = ikmodel.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
    raveLogInfo("\nThe solution for the ik Parameters is ")
    if solutions is not None and len(solutions)>0:
        print solutions
        traj=manipprob.MoveManipulator(solutions[0],execute=False,outputtrajobj=True)
    else:
        RaveLogError("Couldn't find a solution!!!")
raveLogInfo("And on...")
robot.GetController().SetPath(traj)
robot.WaitForController(0) # wait
raveLogInfo("Ta!")
time.sleep(20)