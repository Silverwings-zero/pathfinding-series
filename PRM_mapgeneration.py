import rospy
from sensor_msgs.msg import JointState
import math, random
from mpl.core.plan_sim import Robot_model
from mpl.core.plan_if import movegroup
from mpl.core.trajectory_processing import trajectoryProcess
import time
import pickle
import networkx as nx
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


def PRM(nodeList, numPoint, checkradius):
    #setup PRM map
    for i in range(0, numPoint):
        nodeList.append(Sample())
    for i in nodeList:     
        w = kNearest(i, nodeList, checkradius)
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


def lowestFcost(list):
    currentIndex = 0
    currentVal = list[currentIndex].Fcost
    for i in range(1, len(list)):
        if list[i].Fcost < currentVal:
            currentIndex = i
            currentVal = list[i].Fcost
    return list[currentIndex]


def rrt_m(a, b, maxTrial):
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


def main():
    testpointsset = []
    with open('/root/mpl_example/scripts/standardset.pkl', 'rb') as f:
        testpointsset = pickle.load(f)
    startNode = Node(testpointsset[0][0][0],testpointsset[0][0][1],testpointsset[0][0][2],testpointsset[0][0][3],testpointsset[0][0][4],testpointsset[0][0][5])
    endNode = Node(testpointsset[0][1][0],testpointsset[0][1][1],testpointsset[0][1][2],testpointsset[0][1][3],testpointsset[0][1][4],testpointsset[0][1][5])
    print(startNode)
    print(endNode)
    nodeList = []
    PRM(nodeList, 400, 20)
    G=nx.Graph() 
    G.add_nodes_from(nodeList)
    edges = []
    for i in nodeList:
        for j in i.connectedNodes:
            G.add_edge(i, j, weight = i.dist(j))
    sub_graphs = list(nx.connected_component_subgraphs(G))
    nodeList = list(sub_graphs[0].nodes)
    edgeList = list(sub_graphs[0].edges)
    G.clear()
    G.add_nodes_from(nodeList)
    G.add_edges_from(edgeList)
    nx.write_gpickle(G, "graph")
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
