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

