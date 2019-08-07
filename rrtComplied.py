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

prob = 0.2      #probrability of setting random node to the target
steplength = 0.1
Threshold = 0.11
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

def Nearest(q, nodeList):
    current = None
    for i in nodeList:
        if i is not None:
            current = i
    for i in nodeList:
        if(i is not None and q.dist(i) < q.dist(current)):
            current = i
    return current


def printpath(a):
    k = [a.toList()]
    current = a
    while current.parent is not None:
        k.append(current.parent.toList())
        current = current.parent
    k.reverse()
    return k

def rrt(a, b, maxTrial):
    """[summary]
    rrt finds a path from the given starting and ending position in given 6-D space
    it randomly explore nodes in 6-D space

    Arguments:
        a {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        b {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        maxTrial {[int]}  -- [maximum iterations will be executed before determines the pathfinding to be unproductive]

    Returns:
        [list] -- [the path node went through from starting point to endpoint represented as lists]
        [boolean] -- [whether the pathfinding is productive]
    """
    path = []
    iterCount = 0
    nodeList = [] #list containing all nodes
    qinit = Node(a[0],a[1],a[2],a[3],a[4],a[5])
    qgoal = Node(b[0],b[1],b[2],b[3],b[4],b[5])
    nodeList = [qinit]
    for i in range(0, maxTrial):
        Xrand = None
        p = random.random()
        if(0 < p < prob):
            Xrand = qgoal
        elif(prob < p < 1.0):
            Xrand = Sample()
        Xnear = Nearest(Xrand,nodeList)
        Xnew = Xnear.Extend(Xrand)
        if Xnew is not None:
            nodeList.append(Xnew)
            Xnew.parent = Xnear
            if Xnew.dist(qgoal) < Threshold:
                qgoal.parent = Xnew
                print("True" + str(i))
                #printpath()
                #for i in nodeList:
                #    print i
                nodeList.append(qgoal)
                path = printpath(qgoal)
                return path, True

    print("false")
    return path, False

def rrt_m(a,b,maxTrial):
    """[summary]
    similar with rrt, but it goes towards to exploring nodes until it is no longer traversable

    Arguments:
        a {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        b {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        maxTrial {[int]}  -- [maximum iterations will be executed before determines the pathfinding to be unproductive]

    Returns:
        [list] -- [the path node went through from starting point to endpoint represented as lists]
        [boolean] -- [whether the pathfinding is productive]
    """
    path = []
    iterCount = 0
    nodeList = [] #list containing all nodes
    qinit = Node(a[0],a[1],a[2],a[3],a[4],a[5])
    qgoal = Node(b[0],b[1],b[2],b[3],b[4],b[5])
    nodeList = [qinit]
    for i in range(0, maxTrial):
        Xrand = None
        p = random.random()
        if(0 < p < prob):
            Xrand = qgoal
        elif(prob < p < 1.0):
            Xrand = Sample()
        Xnear = Nearest(Xrand,nodeList)
        Xnew = Xnear.Extend(Xrand)
        while Xnew is not None:
            nodeList.append(Xnew)
            Xnew.parent = Xnear
            if Xnew.dist(qgoal) < Threshold:
                qgoal.parent = Xnew
                print("True" + str(i+1))
                nodeList.append(qgoal)
                path = printpath(qgoal)
                return path, True
            if Xnew.dist(Xrand) < Threshold:
                break
            prev = Xnear
            Xnear = Xnew
            Xnew = Xnew.Extend(Xrand)
    print("False")
    return path, False

def rrt_Connect(a, b, maxTrial):
    """[summary]
    rrt_Connect construct two trees starting from both starting and ending position, a path is found when two tree meets

    Arguments:
        a {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        b {[list]}  -- [a list of six doubles to represent the coordinate of the Node]
        maxTrial {[int]}  -- [maximum iterations will be executed before determines the pathfinding to be unproductive]

    Returns:
        [list] -- [the path node went through from starting point to endpoint represented as lists]
        [boolean] -- [whether the pathfinding is productive]
    """
    qinit = Node(a[0],a[1],a[2],a[3],a[4],a[5])
    qgoal = Node(b[0],b[1],b[2],b[3],b[4],b[5])
    start_Nodelist = [qinit]
    end_Nodelist = [qgoal]
    iterCount = 0
    result = []
    while iterCount < maxTrial:
        qrand = Sample()
        iterCount += 1
        qnearest = Nearest(qrand, start_Nodelist)
        qnew = qnearest.Extend(qrand)
        if qnew is not None and qnew.dist(qrand) > Threshold:
            start_Nodelist.append(qnew)
            
            qnearest1 = Nearest(qnew, end_Nodelist)
            qnew1 = qnearest1.Extend(qnew)
            if qnew1 is not None and qnew1.dist(qnew) > Threshold:
                end_Nodelist.append(qnew1)

                indic = True
                while indic:
                    qnew2 = qnew1.Extend(qnew)
                    if qnew2 is not None:
                        end_Nodelist.append(qnew2)
                        qnew1 = qnew2
                    else:
                        break
                    if qnew1.dist(qnew2) < Threshold:
                        indic = False
            if qnew1 is not None and qnew1.dist(qnew) < Threshold:
                result = printpath(qnew)
                retemp = printpath(qnew1)
                i = len(retemp) - 1
                while i >= 0:
                    result.append(retemp[i])
                    i -= 1

                #result.remove(None)
                print("True" + str(iterCount))
                return result, True
        if len(end_Nodelist) < len(start_Nodelist):
            temp = end_Nodelist
            end_Nodelist = start_Nodelist
            start_Nodelist = temp
    print("False")
    return result, False


def scale(a, b, c):
    tempMax = max(a, b, c)
    tempMin = min(a, b, c)
    dif = tempMax - tempMin
    return (a-tempMin)/dif, (b-tempMin)/dif, (c-tempMin)/dif

def stepEval(k):
    if k < 50:
        return 100
    elif k >= 50 and k < 80:
        return 90
    elif k >= 80 and k < 100:
        return 80
    elif k >= 100 and k < 130:
        return 70
    elif k >= 130 and k < 150:
        return 60
    elif k >= 150:
        return 50

def timeEval(t):
    if t < 0.1:
        return 100
    elif t >= 0.1 and t < 0.5:
        return 90
    elif t >= 0.5 and t < 1:
        return 80
    elif t >= 1 and t < 4:
        return 70
    elif t >= 4 and t < 8:
        return 60
    elif t >= 8:
        return 50

def runtestset(maxTrial,method,testset):
    """[summary]
    this will produce a score that evaluate the effectiveness of the pathfinding method that is running 

    Arguments:
        maxTrial {[int]}  -- [maximum iterations will be executed before determines the pathfinding to be unproductive]
        method {[function]}  -- [which method to execute when running testset]
        testset {[list]}  -- [which testset to use, the path of the testset]

    Returns:
        [double] -- [score that each run of testset gets]
    """
    testpointsset = []
    with open(testset, 'rb') as f:
        testpointsset = pickle.load(f)
    totaltime = 0
    totalstep = 0
    successfulLoopCount = 0
    result = 0
    routeset = []
    datasize = len(testpointsset)
    for i in range(0, datasize):
        print("starting node is: " + str(testpointsset[i][0]))
        print("end node is: " + str(testpointsset[i][1]))
        time_start=time.time()
        route, checker = method(testpointsset[i][0],testpointsset[i][1],maxTrial)
        time_end=time.time()
        timelapse = time_end - time_start

        if not checker:
            result += 0
        else:
            result = result + 50 + stepEval(len(route))* 0.25 + timeEval((timelapse)* 0.25)

        result = result / datasize
    return result*100

def main():
    result = runtestset(1000, rrt_Connect,"standardset.pkl")
    print result
    with open('data','wb') as w:
        pickle.dump(result, w)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
