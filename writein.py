import rospy
from sensor_msgs.msg import JointState
import math, random
from mpl.core.plan_sim import Robot_model
from mpl.core.plan_if import movegroup
from mpl.core.trajectory_processing import trajectoryProcess
import time
import pickle

class Node:
    x1 = 0
    x2 = 0
    x3 = 0
    x4 = 0
    x5 = 0
    x6 = 0
    parent = None

    def __init__(self, x1, x2, x3, x4, x5, x6):
        #super(Node, self).__init__()
        self.x1 = x1
        self.x2 = x2
        self.x3 = x3
        self.x4 = x4
        self.x5 = x5
        self.x6 = x6


    def dist(self, k):
        return math.sqrt((self.x1 - k.x1) ** 2+(self.x2 - k.x2) ** 2+(self.x3 - k.x3) ** 2+(self.x4 - k.x4) ** 2+(self.x5 - k.x5) ** 2+(self.x6 - k.x6) ** 2)


    def toList(self):
        return [self.x1, self.x2, self.x3, self.x4, self.x5, self.x6]


    def Extend(self, b):
        if self is not b:
            k = steplength/self.dist(b)
            js = dict(zip(ctrl_arm1_joints, [self.x1 + (b.x1-self.x1)*k, self.x2 + (b.x2-self.x2)*k, self.x3 + (b.x3-self.x3)*k, self.x4 + (b.x4-self.x4)*k, self.x5 + (b.x5-self.x5)*k, self.x6 + (b.x6-self.x6)*k]))
            #print(robot.checkCollision(js))
            if robot.checkCollision( js ):
                return None
            newNode = Node(self.x1 + (b.x1-self.x1)*k, self.x2 + (b.x2-self.x2)*k, self.x3 + (b.x3-self.x3)*k, self.x4 + (b.x4-self.x4)*k, self.x5 + (b.x5-self.x5)*k, self.x6 + (b.x6-self.x6)*k)
            newNode.parent = self
            return newNode


    def __str__(self):
        return "(" + str(self.x1) + "," + str(self.x2) + "," + str(self.x3) + "," + str(self.x4) + "," + str(self.x5) + "," + str(self.x6) + ")"

robot = Robot_model( "/root/mpl_example/mechmind_abb_model.rbt" ) #create instance of robot collision detection
arm1_group = movegroup(robot, 'tool0')
ctrl_arm1_joints = arm1_group.ctrlable_joints()
arm1_js_name  = ctrl_arm1_joints

def Sample():
    while(True):
        a, b, c, d, e, f = (random.random()-0.5) * 2 * math.pi, (random.random()-0.5) * 2 * math.pi, (random.random()-0.5) * 2 * math.pi, (random.random()-0.5) * 2 * math.pi,(random.random()-0.5) * 2 * math.pi,(random.random()-0.5) * 2 * math.pi
        js = dict(zip(ctrl_arm1_joints, [a,b,c,d,e,f]))
        if not(robot.checkCollision( js )):
            return Node(a,b,c,d,e,f)

testpointsset = []
fileName = "standardset"
for i in range(0,100):
    point = []
    point.append(Sample().toList())
    point.append(Sample().toList())
    testpointsset.append(point)


with open(fileName, 'wb') as f:
    pickle.dump(testpointsset,f)


# with open(fileName, 'rb') as f:
#     testpointsset = pickle.load(f)

print testpointsset