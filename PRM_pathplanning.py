import rospy
from sensor_msgs.msg import JointState
import math, random
from mpl.core.plan_sim import Robot_model
from mpl.core.plan_if import movegroup
from mpl.core.trajectory_processing import trajectoryProcess
import time
import pickle
import networkx as nx
from matplotlib import pyplot as plt
import scipy


class Node:

    def __init__(self, x1, x2, x3, x4, x5, x6):
        #super(Node, self).__init__()
        self.x1 = x1
        self.x2 = x2
        self.x3 = x3
        self.x4 = x4
        self.x5 = x5
        self.x6 = x6
        self.parent = None
        self.connectedNodes = []
        self.Gcost = 0
        self.Hcost = 0
        self.Fcost = 0


    def dist(self, k):
        return math.sqrt((self.x1 - k.x1) ** 2+(self.x2 - k.x2) ** 2+(self.x3 - k.x3) ** 2+(self.x4 - k.x4) ** 2+(self.x5 - k.x5) ** 2+(self.x6 - k.x6) ** 2)


    def toList(self):
        return [self.x1, self.x2, self.x3, self.x4, self.x5, self.x6]


    def Extend(self, b):
        if self is not b:
            k = steplength/self.dist(b)
            js = dict(zip(ctrl_arm1_joints, [self.x1 + (b.x1-self.x1)*k, self.x2 + (b.x2-self.x2)*k, self.x3 + (b.x3-self.x3)*k, self.x4 + (b.x4-self.x4)*k, self.x5 + (b.x5-self.x5)*k, self.x6 + (b.x6-self.x6)*k]))
            if robot.checkCollision( js ):
                return None
            newNode = Node(self.x1 + (b.x1-self.x1)*k, self.x2 + (b.x2-self.x2)*k, self.x3 + (b.x3-self.x3)*k, self.x4 + (b.x4-self.x4)*k, self.x5 + (b.x5-self.x5)*k, self.x6 + (b.x6-self.x6)*k)
            newNode.parent = self
            return newNode

    def isCollidingPath(self, b):
        dist = self.dist(b)
        step = int(dist/steplength)
        temp = self
        for i in range(0, step):
            temp = temp.Extend(b)
            if temp == None:
                return True
        return False

    def __str__(self):
        return "(" + str(self.x1) + "," + str(self.x2) + "," + str(self.x3) + "," + str(self.x4) + "," + str(self.x5) + "," + str(self.x6) + ")"

    def Connect(self, a):
        self.connectedNodes.append(a)
        a.connectedNodes.append(self)

    def initalize(self, startNode, endNode):
        self.Gcost = self.dist(startNode)
        self.Hcost = self.dist(endNode)
        self.Fcost = self.Gcost + self.Hcost

def swapPositions(list, pos1, pos2):  
    list[pos1], list[pos2] = list[pos2], list[pos1] 
    return list

def Connected(a, b):
    if b in a.connectedNodes and a in b.connectedNodes:
        return True
    elif b in a.connectedNodes and a not in b.connectedNodes:
        print("you forgot to add a in b's list")
        return True
    elif b not in a.connectedNodes and a in b.connectedNodes:
        print("you forgot to add b in a's list")
        return True
    else:
        return False

prob = 0.3
steplength = 0.1
Threshold = 0.11
robot = Robot_model( "/root/mpl_example/mechmind_abb_model.rbt" )
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
    current = nodeList[0]
    for i in nodeList:
        if(i is not None and i is not q and q.dist(i) < q.dist(current)):
            current = i
    return current


def ajacentNodewithinRadius(q, nodeList, radius):
    result = []
    for i in nodeList:
        if i is not q and i.dist(q) <= radius:
            result.append(i)
    return result


def kNearest(q, nodeList, checkNodeNum):
    nodeList = bubble_sort(q, nodeList)
    return nodeList[1:checkNodeNum + 1]


def bubble_sort(q, list):
    length = len(list)
    for index in range(length):
        for j in range(1, length - index):
            if list[j - 1].dist(q) > list[j].dist(q):
                list[j - 1], list[j] = list[j], list[j - 1]
    return list


def toNodeList(l):
    result = []
    for i in l: 
        result.append(Node(i[0],i[1],i[2],i[3],i[4],i[5]))
    return result


def printpath(a):
    k = [a]
    current = a
    while current.parent is not None:
        k.append(current.parent)
        current = current.parent
    k.reverse()
    return k

"""[summary]
PRM creates a map to navigate in a space by randomly generating points

Arguments:
    nodelist {[list]}  -- [a list of nodes intended to store the generated nodes]
    numPoint {[int]}  -- [number of points to generate]
    checknum {[int]}  -- [how many points that one point will treat as neighbor]

Returns:
"""
def PRM(nodeList, numPoint, checknum):
    #setup PRM map
    for i in range(0, numPoint):
        nodeList.append(Sample())
    for i in nodeList:     
        w = kNearest(i, nodeList, checknum)
        for j in w:
            if not(i.isCollidingPath(j)):
                i.Connect(j)  
    for i in nodeList:
        i.connectedNodes = list(dict.fromkeys(i.connectedNodes))
    '''
    for i in range(0, len(nodeList)):
        nodeInProximity = ajacentNodewithinRadius(nodeList[i], nodeList, checkradius)
        for j in nodeInProximity:
            print(nodeList[i])
            print(j)
            if not(nodeList[i].isCollidingPath(j)):
                nodeList[i].Connect(j)
        print("***************")
    #initalize costs in PRM map
    for i in nodeList:
        i.connectedNodes = list(dict.fromkeys(i.connectedNodes))
    '''

def steps_normalization(nodelist, steplength):
    result = []
    for i in range(len(nodelist)-1):
        result.append(nodelist[i])
        if nodelist[i].dist(nodelist[i+1]) > Threshold:
            temp = []
            k = int(nodelist[i].dist(nodelist[i+1])/steplength)
            t = nodelist[i]
            for j in range(k):
                if t is not None:
                    t = t.Extend(nodelist[i+1])
                    temp.append(t)
            result += temp
    return result

def lowestFcost(list):
    currentIndex = 0
    currentVal = list[currentIndex].Fcost
    for i in range(1, len(list)):
        if list[i].Fcost < currentVal:
            currentIndex = i
            currentVal = list[i].Fcost
    return list[currentIndex]


def rrt_m(a, b, maxTrial):
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
    qinit = a
    qgoal = b
    print qinit
    print qgoal
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
    qinit = a
    qgoal = b
    print qinit
    print qgoal
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

def aStar(startNode, endNode, nodeList):
    path = []
    openList = [startNode]
    startNode.initalize(startNode, endNode)
    closedList = []
    while(True):
        current = lowestFcost(openList)
        print(current)
        temp = openList.remove(current)
        closedList.append(temp)

        if current is endNode:
            path.append(endNode)
            temp = endNode
            while temp.parent is not None:
                path.append(temp.parent)
            path.reverse()
            return path

        for neighbour in current.connectedNodes:
            if neighbour in closedList:
                pass
            if current.Gcost + current.dist(neighbour) < neighbour.Gcost or neighbour not in openList:
                neighbour.Gcost = current.Gcost + current.dist(neighbour)
                neighbour.Hcost = neighbour.dist(endNode)
                neighbour.Fcost = neighbour.Gcost + neighbour.Hcost
                neighbour.parent = current
                if neighbour not in openList:
                    openList.append(neighbour)


def PRMsolution(startNode, endNode, rrtiterNum, G):
    print(startNode)
    print(endNode)  
    #for i in G.edges:
        #print i
    print("********point generation finished... awaiting to produce subgraph results***********")
    
    print(nx.is_connected(G))
    print("********subgraph result produced, producing connected graph, commencing rrt point connection***********")
    nodeList = list(G.nodes)
    edgeList = list(G.edges)
    rrtStartTarget = Nearest(startNode, nodeList)
    result, checker = rrt_Connect(startNode, rrtStartTarget, rrtiterNum)
    if checker:
        for k in range(0,len(result)-1):
            temp = result[k]
            nextTemp = result[k+1]
            nodeList.append(temp)
            nodeList.append(nextTemp)
            edgeList.append((temp,nextTemp))
    else:
        return [],False
    rrtEndTarget = Nearest(endNode, nodeList)
    result1, checker = rrt_Connect(endNode, rrtEndTarget, rrtiterNum)
    if checker:
        for k in range(0,len(result1)-1):
            temp = result1[k]
            nextTemp = result1[k+1]
            nodeList.append(temp)
            nodeList.append(nextTemp)
            edgeList.append((temp,nextTemp))
    else:
        return [],False
    nodeList = list(dict.fromkeys(nodeList))
    edgeList = list(dict.fromkeys(edgeList))
    t = []
    for i in edgeList:
        t.append((i[0], i[1], i[0].dist(i[1])))
    edgeList = t
    G.add_nodes_from(nodeList)
    G.add_weighted_edges_from(edgeList)

    print("********connected graph generated, is it connected?***********")
    print(nx.is_connected(G))
    '''
    pos=nx.spring_layout(G)
    plt.title('spring_layout')
    nx.draw(G, with_labels=False, font_weight='bold')
    plt.axis('on') 
    plt.xticks([]) 
    plt.yticks([])
    plt.show() 
    '''
    print("********map building finished, generating A* path***********")
    
    path = []
    result = nx.astar_path(G, startNode, endNode)

    print("********normalizing steps***********")
    result = steps_normalization(result, steplength)
    for i in result:
        if i is not None:
            path.append(i.toList())

    return path,True


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
    with open('graph', 'rb') as k:
        G = nx.read_gpickle(k) 
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
        x = Node(testpointsset[i][0][0],testpointsset[i][0][1],testpointsset[i][0][2],testpointsset[i][0][3],testpointsset[i][0][4],testpointsset[i][0][5])
        y = Node(testpointsset[i][1][0],testpointsset[i][1][1],testpointsset[i][1][2],testpointsset[i][1][3],testpointsset[i][1][4],testpointsset[i][1][5])
        route, checker = method(x,y,maxTrial,G)
        time_end=time.time()
        timelapse = time_end - time_start

        if not checker:
            result += 0
        else:
            result = result + 50 + stepEval(len(route))* 0.25 + timeEval((timelapse)* 0.25)

        result = result / datasize
    return result*100


def main():
    result = runtestset(1000, PRMsolution, "standardset.pkl")
    print result


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
