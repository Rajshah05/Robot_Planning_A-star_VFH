#!/usr/bin/env python

import rospy
import numpy
import math
import rosparam

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

thetarobot=0
xrobot=90
yrobot=0
sensor_dicti={}
sensor_list=[]
laserdata=[]
status=[]

map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

map = numpy.reshape(map, (20, 18))


def transimtomap(x, y):
    """
    imput is coordinates of the point in simulator (int/float)
    returns correspoinding row and column in map (tuple)
    """

    r = 9-numpy.floor(y)
    c = 9+numpy.floor(x)
    r = int(r)
    c = int(c)
    mapcor = (r, c)
    return mapcor


def tranmaptosim(row, col):
    """
    input is row and col in map (int)
    output is node coordinate in simulator (tuple)
    """

    xcor = col-8.5
    ycor = 9.5-row
    simcor = (xcor, ycor)
    return simcor





def hcost(node, endmap):
    """
    input is map node row and column (tuple)
    returns hcost of the node (float)
    """
   
    hcost = numpy.sqrt((numpy.square(node[1]-endmap[1]))+(numpy.square(node[0]-endmap[0])))
    return hcost


open = {}
closed = {}
pathlist = []


def astar(startmap, endmap):
    open[startmap] = [hcost(startmap, endmap), 0]
    while True:
        mintcost = 10000000000000000
        tcost = 0
        for m in open.values():
            tcost = m[0]+m[1]
            if tcost < mintcost:
                    mintcost = tcost
        for k in open:
            if sum(open[k])==mintcost:
                current=k
        del(open[current])                                                                                                                                                           
        closed[current]=mintcost                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        pathlist.append(current)                                                                                                                                                                                                                                                                                                                                                                                                  
        if current==endmap:
            break                                                                                                                                                                                                                                                                                                  
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if (0<=(current[0]+i)<=19 and 0<=(current[1]+j)<=17):
                    if map[current[0]+i][current[1]+j]==0:
                        if (current[0]+i,current[1]+j) not in closed:
                            if (current[0]+i,current[1]+j) not in open:
                                if ((i+j)%2) ==0:
                                    gcost=1.4
                                else:
                                    gcost=1.0
                                open[(current[0]+i,current[1]+j)]=[hcost((current[0]+i,current[1]+j), endmap),gcost]
                            else:
                                if ((i+j)%2)==0:
                                    gcost=1.4
                                else:
                                    gcost=1.0
                                if sum(open[(current[0]+i,current[1]+j)])>(hcost((current[0]+i,current[1]+j), endmap)+gcost):
                                    open[(current[0]+i,current[1]+j)]=[hcost((current[0]+i,current[1]+j), endmap),gcost]


    path=pathlist
    startmap_np=path[0]
    endmap_np=path[-1]
    new_path=[]
    rn=endmap_np
    while rn!=startmap_np:
        new_path.append(rn)
        temp=[]
        temp_cost=[]
        for i in [(-1, -1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
            if (rn[0]+i[0],rn[1]+i[1]) in path:
                temp.append((rn[0]+i[0],rn[1]+i[1]))
                temp_cost.append(closed[(rn[0]+i[0],rn[1]+i[1])])
        ind=temp_cost.index(max(temp_cost))
        rn=temp[ind]
        path.remove(rn)
    new_path.append(startmap_np)
    new_path.reverse()
    return new_path






def callback_robotcpose(msg):
    global thetarobot
    global xrobot
    global yrobot

    xrobot = msg.pose.pose.position.x
    yrobot = msg.pose.pose.position.y

    worirobot = msg.pose.pose.orientation.w
    xorirobot = msg.pose.pose.orientation.x
    yorirobot = msg.pose.pose.orientation.y
    zorirobot = msg.pose.pose.orientation.z
    roll = pitch = yaw = 0.0
    orientation_list = [xorirobot, yorirobot, zorirobot, worirobot]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    thetarobot = yaw




def callback_laserscan(msg):
    global status
    global sensor_list
    global sensor_dicti
    global laserdata
    laserdata = list(msg.ranges)
    R1 = laserdata[0:10]
    R2 = laserdata[10:20]
    R3 = laserdata[20:30]
    R4 = laserdata[30:40]
    R5 = laserdata[40:50]
    R6 = laserdata[50:60]
    R7 = laserdata[60:70]
    FR1 = laserdata[70:80]
    FR2 = laserdata[80:90]
    FR3 = laserdata[90:100]
    FR4 = laserdata[100:110]
    FR5 = laserdata[110:120]
    FR6 = laserdata[120:130]
    FR7 = laserdata[130:140]
    F1 = laserdata[140:150]
    F2 = laserdata[150:160]
    F3 = laserdata[160:170]
    F4 = laserdata[170:180]
    F5 = laserdata[180:190]
    F6 = laserdata[190:200]
    F7 = laserdata[200:210]
    FL1 = laserdata[210:220]
    FL2 = laserdata[220:230]
    FL3 = laserdata[230:240]
    FL4 = laserdata[240:250]
    FL5 = laserdata[250:260]
    FL6 = laserdata[260:270]
    FL7 = laserdata[270:280]
    L1 = laserdata[280:290]
    L2 = laserdata[290:300]
    L3 = laserdata[300:310]
    L4 = laserdata[310:320]
    L5 = laserdata[320:330]
    L6 = laserdata[330:340]
    L7 = laserdata[340:350]
    L8 = laserdata[350:361]

    sensor_list=[R1,R2,R3,R4,R5,R6,R7,FR1,FR2,FR3,FR4,FR5,FR6,FR7,F1,F2,F3,F4,F5,F6,F7,FL1,FL2,FL3,FL4,FL5,FL6,FL7,L1,L2,L3,L4,L5,L6,L7,L8]
    status=laserdata[:]
    sensor_dict={}
    sensor_dicti={}
    
    angle=87.5
    for p in sensor_list:
        count=0
        for j in p:
            if j<0.7:
                count+=1
        sensor_dict[angle]=[count,9999]
        angle=angle-5
    sensor_dicti=sensor_dict

def target_direction(target_node):
    global yrobot
    global xrobot
    slope=math.atan2((target_node[1]-yrobot),abs(target_node[0]-xrobot))
    if slope<0 and target_node[0]-xrobot<0:
        slope=-(3.14159265359+slope)
    if slope>0 and target_node[0]-xrobot<0:
        slope=(3.14159265359-slope)
    return slope


def merge():
    global laserdata
    global status
    global thetarobot
    global yrobot
    global xrobot
    global sensor_dicti
    global sensor_list
    rospy.init_node('merge', anonymous=True)
   
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_robotcpose) 
    rospy.Subscriber('/base_scan', LaserScan, callback_laserscan)
    
    goal_node_x = rospy.get_param('/goalx')
    goal_node_y = rospy.get_param('/goaly')

    path=astar(transimtomap(-8.0,-2.0),transimtomap(goal_node_x,goal_node_y))
    path_cor=[]
    for r in path:
        path_cor.append(tranmaptosim(r[0],r[1]))
  
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5)
    vel_msg=Twist()


    while not rospy.is_shutdown():
        for y in path_cor[0:(len(path_cor)-1)]:
            current_node=y
            next_node=path_cor[path_cor.index(y)+1]
            pchosen_dir=thetarobot
            while math.sqrt((xrobot - next_node[0])**2 + (yrobot - next_node[1])**2)>0.4:
                th=2
                a=10
                b=7
                c=7
                min_cost=99999
                for g in sensor_dicti:
                    if sensor_dicti[g][0]<th:
                        sensor_dicti[g][1]=(a*abs((thetarobot-math.radians(g))-target_direction(next_node)))+(b*abs((thetarobot-math.radians(g))-thetarobot))+(c*abs((thetarobot-math.radians(g))-pchosen_dir))
                        if sensor_dicti[g][1]<min_cost:
                            min_cost=sensor_dicti[g][1]
                
                for b in sensor_dicti:
                    if sensor_dicti[b][1]==min_cost:
                        pchosen_dir=b
                # print(sensor_dicti.values())
             
                if thetarobot>(thetarobot-math.radians(pchosen_dir)):
                    vel_msg.angular.z=-0.5
                    vel_msg.linear.x=0.15
                    pub.publish(vel_msg)
                if thetarobot<(thetarobot-math.radians(pchosen_dir)):
                    vel_msg.angular.z=0.5
                    vel_msg.linear.x=0.15
                    pub.publish(vel_msg)
                
                vel_msg.linear.x=0.4
                pub.publish(vel_msg)
                if len(status)>10:
                    while min(status[0:361])<0.5:
                        vel_msg.linear.x=0
                        vel_msg.angular.z=-0.5
                        pub.publish(vel_msg)
                if math.sqrt((xrobot - path_cor[-1][0])**2 + (yrobot - path_cor[-1][1])**2)<0.4:
                    rospy.signal_shutdown("Reached the Goal !!!")

                print(next_node)
                
                
        rospy.spin()


if __name__ == '__main__':
    merge()