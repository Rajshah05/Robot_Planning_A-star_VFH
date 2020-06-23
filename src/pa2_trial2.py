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
xrobot=0
yrobot=0
sensor_dicti={}
sensor_list=[]
laserdata=[]

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


# x = rospy.get_param('/goalx')
# print(x)


def hcost(node, endmap):
    """
    input is map node row and column (tuple)
    returns hcost of the node (float)
    """
    # gcost=numpy.sqrt((numpy.square(node[1]-startmap[1]))+(numpy.square(node[0]-startmap[0])))
    hcost = numpy.sqrt((numpy.square(node[1]-endmap[1]))+(numpy.square(node[0]-endmap[0])))
    # fcost=gcost+hcost
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
        # closed_copy=closed.copy()                                                                                                                                                                                                                                                                                                                                  
        # for l in closed_copy:                                                                                                                                                                                        
        #      if cost(current,startmap,endmap)<closed_copy[l]:
        #           del(closed[l])
        #closed[current]=mintcost
        # closed_copy=closed.copy()
        # for n in closed_copy:
        #     if closed[n]>mintcost:
        #         del(closed[n])                                                                                                                                                               
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
                                    # print('cost greater')
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

# print(astar(transimtomap(-8.0,-2.0),transimtomap(4.5,9.0)))





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
    # if thetarobot<0:
    #     thetarobot=(math.pi)+(math.pi+thetarobot)
    # print xrobot
    # print yrobot
    # print thetarobot
    # return [xrobot,yrobot,thetarobot]

# def target_direction(currentNode,nextNode):
#     slope=numpy.arctan((nextNode(1)-currentNode(1))/abs(nextNode(0)-currentNode(0)))
#     if slope<0:
#         slope=(math.pi)+(math.pi+thetarobot)
#     return round(slope,1)



def callback_laserscan(msg):
    global sensor_list
    global sensor_dicti
    # global status
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
    sensor_dict={}
    sensor_dicti={}
    angle=round(87.5*math.pi/180,2)
    for p in sensor_list:
        count=0
        for j in p:
            if j<3:
                count+=1
        sensor_dict[angle]=[count,99999]
        angle=angle-round(5*math.pi/180,2)
    sensor_dicti=sensor_dict
    # print sensor_dicti
    # print(sensor_list)
    # return sensor_dict

    # th=6
    # for g in sensor_dict:
    #     if sensor_dict[g][0]<th:
    #         sensor_dict[g][1]=(a*abs(g-target_direction()))+(b*abs(g-thetarobot)+(c*abs(g-previous_direction()))
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
    global thetarobot
    global xrobot
    global yrobot
    global sensor_dicti
    global sensor_list
    rospy.init_node('merge', anonymous=True)
    rospy.Subscriber('/base_scan', LaserScan, callback_laserscan)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_robotcpose)

    
    goal_node_x = rospy.get_param('/goalx')
    goal_node_y = rospy.get_param('/goaly')

    path=astar(transimtomap(-8.0,-2.0),transimtomap(goal_node_x,goal_node_y))
    path_cor=[]
    for r in path:
        path_cor.append(tranmaptosim(r[0],r[1]))
    print(path_cor)
  
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1000)
    vel_msg=Twist()

    print(target_direction((-6.5,-0.5))
    # print (xrobot)

    # while not rospy.is_shutdown():
    #     for y in path_cor[0:(len(path_cor)-1)]:
    #         current_node=y
    #         next_node=path_cor[path_cor.index(y)+1]
        
    #         while math.sqrt((xrobot - next_node[0])**2 + (yrobot - next_node[1])**2)>0.7:
    #             # print(next_node)
    #             # slope=math.atan2((next_node[1]-yrobot),abs(next_node[0]-xrobot))
    #             # if slope<0 and next_node[0]-xrobot<0:
    #             #     slope=-(3.14159265359+slope)
    #             # if slope>0 and next_node[0]-xrobot<0:
    #             #     slope=(3.14159265359-slope)

        
    #             # vel_msg=Twist()
    #             # while abs(thetarobot-slope)>=0.15:                                                
    #             #     vel_msg.angular.z=-0.1
    #             #     pub.publish(vel_msg)
    #             #     print(thetarobot)
                
           
                
    #             # while True:
    #             #     print 1
    #             #     vel_msg2.angular.z=0.2
    #             #     vel_msg2.linear.x=0.2
    #             #     pub.publish(vel_msg2)

    
    #             th=2
    #             a=0.2
    #             b=0.5
    #             c=0.8
    #             min_cost=99999                                                                                                                                                                                          
    #             pchosen_dir=thetarobot
    #             for g in sensor_dicti:
    #                 # print('AAAAAAAAAAAA')
    #                 if sensor_dicti[g][0]<th:
    #                     sensor_dicti[g][1]=(a*abs(g-target_direction(next_node)))+(b*abs(g-thetarobot))+(c*abs(g-pchosen_dir))
    #                     if sensor_dicti[g][1]<min_cost:
    #                         min_cost=sensor_dicti[g][1]
                
    #             for b in sensor_dicti:
    #                 if sensor_dicti[b][1]==min_cost:
    #                     pchosen_dir=b
    #             # print(b)
    #             # print(sensor_dicti)
    #             # print(sensor_list)
    #             # print thetarobot
    #             # print pchosen_dir
    #             while pchosen_dir>0:
    #                 print pchosen_dir
    #                 # print sensor_dicti
    #                 vel_msg.angular.z=-0.3
    #                 vel_msg.linear.x=0.1
    #                 pub.publish(vel_msg)
    #                 print target_direction(next_node)
    #                 for g in sensor_dicti:
    #                     if sensor_dicti[g][0]<th:
    #                         sensor_dicti[g][1]=(a*abs(g-target_direction(next_node)))+(b*abs(g-thetarobot))+(c*abs(g-pchosen_dir))
    #                         if sensor_dicti[g][1]<min_cost:
    #                             min_cost=sensor_dicti[g][1]
    #                 for b in sensor_dicti:
    #                     # print("BBBBBBBBBBBBBBBBBBBBBBBBB")
    #                     if sensor_dicti[b][1]==min_cost:
    #                         pchosen_dir=b
    #             # print pchosen_dir

    #             while pchosen_dir<0:
    #                 print pchosen_dir
    #                 # print sensor_dicti
    #                 vel_msg.angular.z=0.3
    #                 vel_msg.linear.x=0.1
    #                 pub.publish(vel_msg)
    #                 print target_direction(next_node)
    #                 for g in sensor_dicti:
    #                     if sensor_dicti[g][0]<th:
    #                         sensor_dicti[g][1]=(a*abs(g-target_direction(next_node)))+(b*abs(g-thetarobot))+(c*abs(g-pchosen_dir))
    #                         if sensor_dicti[g][1]<min_cost:
    #                             min_cost=sensor_dicti[g][1]
    #                 for b in sensor_dicti:
    #                     print('BBBBBBBBBBBBBBBBBBB')
    #                     if sensor_dicti[b][1]==min_cost:
    #                         pchosen_dir=b




    #             # if b>0:
    #             #     vel_msg=Twist()
    #             #     print thetarobot
    #             #     while abs(thetarobot)>abs(thetarobot-b):
    #             #         vel_msg.angular.z=-0.2
    #             #         vel_msg.linear.x=0.1
    #             #         pub.publish(vel_msg)

    #             # if b<0:
    #             #     vel_msg1=Twist()
    #             #     while abs(thetarobot)<abs(thetarobot-b):
    #             #         vel_msg1.angular.z=0.2
    #             #         vel_msg1.linear.x=0.1
    #             #         pub.publish(vel_msg1)            
    #             # t0 = rospy.Time.now().to_sec()
    #             # current_angle=0
    #             # vel_msg1=Twist()
    #             # while current_angle < ((30*math.pi)/180):
    #             #     vel_msg1.angular.z=-0.2
    #             #     pub.publish(vel_msg1)
    #             #     t1 = rospy.Time.now().to_sec()
    #             #     current_angle = abs(vel_msg1.angular.z)*(t1-t0)
    #             #     # print(current_angle)
    #             #     # print((30*math.pi)/180)
    #             #     print(t0)
    #             #     print(t1)

    #             # vel_msg2=Twist()
    #             # vel_msg2.linear.x=0.3
    #             # if math.sqrt((xrobot - next_node[0])**2 + (yrobot - next_node[1])**2)<=0.7:
    #             #     continue
            
    #         # rate.sleep()
        
    #     rospy.spin()



if __name__ == '__main__':
    merge()

