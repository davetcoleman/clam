import time
import os

from openravepy import *
from numpy import *
from math import *

import traceback
#import crustCrawler
import pdArm
import joint

def buildTransform(x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
    """
    Convert the specified x,y,z translation and roll,pitch,yaw rotation into a 4x4 trasformation matrix
    
    The trasformations are applied in the following order: (1) roll, (2) pitch, (3) yaw, (4) translation.  the result is treurned in a 4x4 numpy.Array suitable for use with OpenRAVE.  The function decomposeTransform in this module converts a transformation matrix to x,y,z,roll,pitch,yaw.
    """
    rx = roll
    ry = pitch
    rz = yaw
    translation = matrix([[1,0,0,x],
                          [0,1,0,y],
                          [0,0,1,z],
                          [0,0,0,1]])

    rotx = matrix([[1,      0,         0, 0],
                   [0, cos(rx), -sin(rx), 0],
                   [0, sin(rx),  cos(rx), 0],
                   [0,      0,         0, 1]])

    roty = matrix([[ cos(ry), 0, sin(ry), 0],
                   [       0, 1,       0, 0],
                   [-sin(ry), 0, cos(ry), 0],
                   [       0, 0,       0, 1]])

    rotz = matrix([[cos(rz), -sin(rz), 0, 0],
                   [sin(rz),  cos(rz), 0, 0],
                   [      0,        0, 1, 0],
                   [      0,        0, 0, 1]])
    
    return asarray(translation*rotz*roty*rotx)
    
def decomposeTransform(t):
    """
    Convert the specified 4x4 trasformation matrix rotation into an x,y,z translation and roll,pitch,yaw
    
    The trasformations are applied in the following order: (1) roll, (2) pitch, (3) yaw, (4) translation.  The function decomposeTransform in this module converts a transformation matrix to x,y,z,roll,pitch,yaw.
    """
    t.shape = (t.size/4,4)
    x=t[0,3]
    y=t[1,3]
    z=t[2,3]
    err=10**-4
    if abs(t[0,0]) > err:
        yaw = atan2(t[1,0], t[0,0])
    elif abs(t[1,0]) > err:
        yaw = pi/2 if t[1,0] > 0 else 3*pi/2
    else: # in this case, pitch = pi/2 or 3*pi/2, and the roll axis is transformd onto the yaw axis, therefore yaw and roll cannot be seperated, so we attribute all the combined yaw and roll to yaw
        # sin = -t[0,1]
        # cos = t[1,1]
        #print 'sin:',-t[0,1],'  cos:',t[1,1]
        yaw = atan2(-t[0,1],t[1,1])
        roll = 0
        
    pitch = atan2(-t[2,0], sqrt(t[2,1]**2+t[2,2]**2))
    if abs(t[2,2]) > err:
        roll = atan2(t[2,1],t[2,2])
    elif abs(t[2,1]) > err:
        roll = pi/2 if t[2,1] > 0 else 3*pi/2
    else:
        roll = 0 #pitch = pi/2 or 3*pi/2, so we attribute all the combined yaw and roll to yaw
    
    return x,y,z,roll,pitch,yaw

def loadRobot(env,robotFile):
    robot = env.ReadRobotXMLFile(robotFile)
    env.AddRobot(robot)
    return robot
class Arm(object):
    def __init__(self):
        self.env = None
        self.realArm = None
        self.robot = None
        self.b = None
        self.t = None
        self.locked=False
        self.sparseness=1
        self.fingersClosed = False
    
    def sim(self, envFile='src/rave_experimental/arm/test_env.env.xml',robotFile='src/rave_experimental/arm/pdArm.robot.xml', absPath = False):
    	
    	if not absPath:
			# resolve file names
			envFile = os.path.join(os.getcwd(), envFile)
			robotFile = os.path.join(os.getcwd(), robotFile)
    	
        global env,r,b,t0,arm
        if self.env == None:
            self.env = Environment() # create openrave environment
            self.env.SetViewer('qtcoin') # attach viewer (optional)
        else:
            self.env.Reset()
        if envFile:
            self.env.Load(envFile)
        try:
            self.robot = loadRobot(self.env, robotFile)
            if self.robot.GetManipulators():
                self.robot.SetActiveDOFs(self.robot.GetManipulatopythrs()[0].GetArmIndices())
        except Exception as e:
            print e
        
        self.env.SetCollisionChecker(self.env.CreateCollisionChecker('ode'))
        if self.robot.GetManipulators():
            try:
#                ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.databases.inversekinematics.IkParameterization.Type.Translation3D)
                ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.databases.inversekinematics.IkParameterization.Type.Transform6D)
                if not ikmodel.load():
#                    ikmodel.generate(freejoints=[6])
                    ikmodel.autogenerate()
            except Exception as e:
                print e
                traceback.print_exc()
        
        try:
            self.b = openravepy.interfaces.BaseManipulation(self.robot)
            self.t = openravepy.interfaces.TaskManipulation(self.robot)
        except Exception as e:
            print e
            traceback.print_exc()
        
        if self.realArm != None:
            deg = self.realArm.read_position()
            rad = list((pi*x/180.0 if x != None else None) for x in deg)
            
            current=self.robot.GetDOFValues()
            
            for i in range(len(rad)):
                if rad[i] == None:
                    rad[i]=current[i]
            
            self.robot.GetController().SetDesired(rad)
    
    def real(self, dev='/dev/ttyUSB0'):
        try:
            self.realArm = pdArm.PdArm(dev_name=dev)
            self.locked=False
            if self.robot != None:
                self.grabRealJoints()
        except RuntimeError:
            pass
        except Exception as e:
            traceback.print_exc(e)
        
        
    def runTrajectory(self,trajData):
        
#        print trajData
        
        data = trajData.split()
        print data
        n,joints,options=map(int,data[:3])
        data=data[3:]
        if options == 13:
            total= joints + 8
        else:
            total = len(data)/n
            print 'unknown trajectory options',options,' Guessing total is',total
        print n,joints,total
        
        traj=[]
        while len(data) >= total:
            traj=traj+[list( float(i)*180/pi for i in data[1:joints+1])]
            if traj[-1]:
                traj[-1][-1]=None
#                traj[-1][0]=None
            else:
                traj=traj[:-1]
#            print data[0]
            data = data[total:]
        
        traj=list(traj[i] for i in range(len(traj)) if i%self.sparseness==0)# or i>len(traj)-20)
        
#        print traj
        
        if self.realArm:
            self.realArm.trajectory(traj)
        
        #openravepy_int.Trajectory(trajData, self.robot)
            
        return traj
    
    def joints(self,joints):
        if self.env == None or self.robot == None or self.b == None :
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        if not self.locked:
            self.lock()
        
        if self.realArm:
            self.grabRealJoints()
        
        newJoints = self.robot.GetActiveDOFValues()
        indices = self.robot.GetActiveDOFIndices()
        
        print joints
        print indices
        print newJoints
        
        for i in range(len(newJoints)):
#            print i
            if joints[indices[i]] != None:
                newJoints[i] = joints[indices[i]]
        
        result = self.b.MoveActiveJoints(newJoints,outputtraj=True)
        
        self.runTrajectory(result)
#        return self.runTrajectory(result)
        
    def getJoints(self):
        if self.env == None or self.robot == None or self.b == None :
            self.sim()
        
        return self.robot.GetActiveDOFValues()
        
    def moveTo(self,x,y,z,roll=0,pitch=0,yaw=0,constrain=False):
        if self.env == None or self.robot == None or self.b == None:
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        if not self.locked:
            self.lock()
        
        constraintError = None
        constraintMatrix = None
        constraintFreedoms = None
        
        if constrain:
            constraintFreedoms = array([0]*6)
            newpos = [x,y,z,roll,pitch,yaw]
            oldpos = list(self.getPose())
            
            for i in range(6):
                if abs(newpos[i]-oldpos[i] > .001):
                    constraintFreedoms[i] = 1
                    constraintError = 0.001
                    constraintMatrix = eye(4)
            
            
        #result = self.b.MoveToHandPosition(translation=[x,y,z],outputtraj=True)
#        result = self.b.MoveToHandPosition(translation=[x,y,z],rotation=[axisx,axisy,axisz,turn],outputtraj=True)
        t = buildTransform(x,y,z,roll,pitch,yaw)
        #print t
        #raw_input("press enter . . .")
        result = self.b.MoveToHandPosition([t],outputtraj=True,constraintfreedoms=constraintFreedoms,constraintmatrix=constraintMatrix,constrainterrorthresh=constraintError)
        #self.robot.WaitForController(0)
        
        resultList = result.split("   ")
        resultList = resultList[:len(resultList)/2]
        
        self.runTrajectory(result)
#        return self.runTrajectory(result)
        
    
    def getPose(self):
        t = self.robot.GetManipulators()[0].GetEndEffectorTransform()
        return decomposeTransform(t)
    
    def home(self):
        if self.env == None or self.robot == None or self.b == None :
            self.sim()
        
        return self.joints([0]*self.robot.GetDOF())
    
    def zero(self):
        if self.env == None or self.robot == None:
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        self.robot.GetController().SetDesired([0]*self.robot.GetDOF())
        
        if self.realArm:
            self.realArm.zero()
        
    def close(self):
        if self.env == None or self.robot == None:
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        self.t.CloseFingers()
        
        if self.realArm:
            self.realArm.claw_soft_close()
        
        self.robot.WaitForController(0)
        grabList = list(b for b in self.env.GetBodies() if self.env.CheckCollision(b,self.robot) and b.GetName() != 'floor')
        
        if len(grabList) == 1:
            self.robot.Grab(grabList[0])
        
        self.fingersClosed = True
        
    def open(self):
        if self.env == None or self.robot == None:
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        self.t.ReleaseFingers()
        self.robot.ReleaseAllGrabbed()
        
        if self.realArm:
            self.realArm.claw_open()
        
        self.fingersClosed = False
    
    def grabRealJoints(self):
        deg = self.realArm.read_position()
        rad = list((pi*x/180.0 if x != None else None) for x in deg)
        
        current=self.robot.GetDOFValues()
        
        for i in range(len(rad)):
            if rad[i] == None:
                rad[i]=current[i]
        
        self.robot.GetController().SetDesired(rad)
        
    def free(self):
        if self.env == None or self.robot == None:
            self.sim()
        
        if self.realArm == None:
            self.real()
        
        #send twice to work around servo bugs
        self.realArm.set_torque_all(False)
        self.realArm.set_torque_all(False)
        
        stop=False
        
        while not stop:
                
            try:
                self.grabRealJoints()
                
                time.sleep(.01)
            except KeyboardInterrupt:
                print
#                traceback.print_exc()
#                print deg
#                self.realArm.position(deg)
                stop = True
                #send twice to work around servo bugs
                self.realArm.set_torque_all(True)
                self.realArm.set_torque_all(True)
                self.grabRealJoints()
    
    def release(self):
        if self.realArm == None:
            self.real()
        
        #send twice to work around servo bugs
        self.realArm.set_torque_all(False)
        self.realArm.set_torque_all(False)
        self.locked=False
        
    def lock(self):
        if self.realArm == None:
            self.real()
        
        if self.realArm != None:
            #send twice to work around servo bugs
            self.realArm.set_torque_all(True)
            self.realArm.set_torque_all(True)
            
            self.grabRealJoints()
        
        self.locked=True
        
if __name__ == '__main__':
    pass
