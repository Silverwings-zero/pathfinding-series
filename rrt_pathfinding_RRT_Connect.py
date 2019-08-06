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


def rrt_Connect(a, b, maxTrial):
    qinit = Node(a[0],a[1],a[2],a[3],a[4],a[5])
    qgoal = Node(b[0],b[1],b[2],b[3],b[4],b[5])
    print(qinit)
    print(qgoal)
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

def runtestset(maxTrial):
    testpointsset = []
    with open('standardset.pkl', 'rb') as f:
        testpointsset = pickle.load(f)
    totaltime = 0
    totalstep = 0
    successfulLoopCount = 0
    result = []
    routeset = []
    datasize = len(testpointsset)
    for i in range(0, datasize):
        time_start=time.time()
        route, checker = rrt_Connect(testpointsset[i][0],testpointsset[i][1],maxTrial)
        time_end=time.time()
        timelapse = time_end-time_start
        if checker:
            successfulLoopCount += 1
            totaltime += timelapse
            totalstep += len(route)
            routeset.append(route)
        print('totally time spent running',timelapse, 'seconds')
    avgtime = totaltime/successfulLoopCount
    avgstep = totalstep/successfulLoopCount
    successrate = float(successfulLoopCount) * 100 / datasize
    return avgtime, avgstep, successrate, routeset
'''
def main():
    avgtime, avgstep, successrate, routeset = runtestset(2000)
    print(avgstep)
    print(avgtime)
    print(successrate)
    with open('data','wb') as w:
        pickle.dump(routeset, w)


def main():
    maxTrial = 1000
    supertemp = []
    for i in range(1,8):
        biggertemp = []
        for i in range(0, 10):
            temp = []
            avgtime, avgstep, successrate = runtestset(maxTrial)
            temp.append(avgtime)
            temp.append(avgstep)
            temp.append(successrate)
            biggertemp.append(temp)
        maxTrial += 1000
        supertemp.append(biggertemp)
    with open('data','wb') as w:
        pickle.dump(supertemp, w)
'''

def main():
    path, checker = rrt_Connect([2.31999568287,0.963528173386,-2.36286110977,0.601828624877,-1.21129735866,2.58115541867],
        [2.75129714618,-0.962121510088,2.71926859476,-2.12425928957,-1.38390091405,2.00577493086], 10000)
    for i in path:
        print i
    '''
    testpointsset = []
    with open('/root/mpl_example/scripts/standardset', 'rb') as f:
        testpointsset = pickle.load(f)
    path, checker = rrt_Connect(testpointsset[1][0],testpointsset[1][1],1000)
    if checker:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(3) # 10hz
        counter = 0
        while not rospy.is_shutdown() and counter < len(path) is not None:
            print counter
            a = JointState()
            a.header.seq = counter
            a.header.stamp = rospy.Time.now()
            a.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            temp = path[counter]
            print(temp)
            a.position = temp
            rospy.loginfo(a)
            pub.publish(a)
            rate.sleep()
            counter+=1
    else:
        print(checker)
    '''

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass