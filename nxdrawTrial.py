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
            #print(robot.checkCollision(js))
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

a = Node(1,1,1,1,1,1)
b = Node(2,2,2,2,2,2)

nodes=[a,b,2,3,4,5,'a','b','c'] 
edges=[(a,b),(a,5),(b,2),(b,4),(2,b),(2,4),('a','b'),('b','c'),('c','a')] 
G=nx.Graph() 
G.add_nodes_from(nodes) 
G.add_edges_from(edges) 
 
pos=nx.spring_layout(G) 
 
plt.title('spring_layout') 
nx.draw(G, with_labels=True, font_weight='bold') #Draw the graph G with Matplotlib. 
plt.axis('on') 
plt.xticks([]) 
plt.yticks([]) 
 
plt.show() 