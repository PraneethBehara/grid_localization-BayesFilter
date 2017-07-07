#!/usr/bin/env python
import rospy
import rosbag
import math
import tf
import roslib
from std_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
from visualization_msgs.msg import Marker
import rviz
import rospkg
import matplotlib.pyplot as mplot

marker_p = rospy.Publisher('/visualization_marker',Marker,queue_size=100)

trans_var_noise = 10.0
trans_std = math.sqrt(trans_var_noise)
rot_var_noise = 45
rot_std = math.sqrt(rot_var_noise)
gaussian_trans= 1.0/(math.sqrt(2*math.pi)*trans_var_noise)
gaussian_rot = 1.0/(math.sqrt(2*math.pi)*rot_var_noise)
G = np.zeros((35,35,4)) #tetha discretized to 90 degrees
tag = np.ndarray((6,),dtype=np.object)
G[11][27][3] = 1.0
G_cen= np.ndarray(G.shape,dtype=np.object)

class grid:

    def init_grid():
        for i in range(G.shape[0]):
            for j in range(G.shape[1]):
                for k in range(G.shape[2]):
                    G_cen[i][j][k] = class_gcnts(i*20+10,j*20+10,k*90)
        #print "--------Grid initialized--------"

class class_gcnts:  #for centers

    def __init__(self,x=0,y=0,z=0):
        self.x = x
    	self.y = y
    	self.z = z

   	def __str__(self):
    	return "("+str(self.x)+","+str(self.y)+","+str(self.z)+")"
       	pass

class localize:

    def run():
        bag = rosbag.Bag('/home/chinnu/catkin_ws/src/lab4/grid.bag')
        counter = 1
        try:
            for topic, msg, t in bag.read_messages(topics =['Movements', 'Observations']):
                if topic == 'Movements':
                    rot1,trans,rot2 = msg.rotation1,msg.translation*100,msg.rotation2
                    #Normalizing angles to 0,2pi
                    rot1 = normalize(np.degrees(tf.transformations.euler_from_quaternion([rot1.x,rot1.y,rot1.z,rot1.w])[2]))
                    rot2 = normalize(np.degrees(tf.transformations.euler_from_quaternion([rot2.x,rot2.y,rot2.z,rot2.w])[2]))
                    #print rot1,trans,rot2
                    predict(rot1,trans,rot2)
                else:
                    range_tag,bearing,tagNum = msg.range*100,msg.bearing,msg.tagNum
                    #print range_tag,bearing,tagNum
                    bearing = np.degrees(tf.transformations.euler_from_quaternion([bearing.x,bearing.y,bearing.z,bearing.w])[2])
                    update(range_tag,bearing,tagNum)
                    counter += 1
        finally:
            bag.close()

    def get_rot(line_angle,cell_pose):
        req_angle = np.min([np.abs(line_angle - cell_pose), (360 - np.max([cell_pose,line_angle]) + np.min([cell_pose,line_angle]))])
        return req_angle


    def normalize(theta):
        if theta < 0:
            theta = theta + 360
            return theta
        else:
            return theta

    def pub_markers(u,v,w):
        print "---------Publishing Markers-----------"
        rate = rospy.Rate(10)
        lo_marker = Marker()
        lo_marker.header.frame_id = 'map'
        lo_marker.header.stamp = rospy.Time.now()
        lo_marker.ns = 'localization'
        lo_marker.id = 0
        lo_marker.type = Marker.LINE_LIST
        lo_marker.action = Marker.ADD
        lo_marker.scale.x = 0.1
        lo_marker.scale.y = 0.1
        lo_marker.scale.z = 0.1
        line_color = ColorRGBA()
        line_color.r = 0.0
        line_color.g = 0.0
        line_color.b = 1.0
        line_color.a = 1.0
        lo_marker.pose.position.x = u
        lo_marker.pose.position.y = v
        lo_marker.pose.position.z = 0
        ang = tf.transformations.quaternion_from_euler(0,0,w)
        lo_marker.pose.orientation.x = ang[0]
        lo_marker.pose.orientation.y = ang[1]
        lo_marker.pose.orientation.z = ang[2]
        lo_marker.pose.orientation.w = ang[3]
        lo_marker.action = Marker.ADD
        marker_p.publish(lo_marker)
        tag_poses()
        rate.sleep()
    
    def predict(rot1,trans,rot2):
        global G,G_cen
        G_new = np.zeros(G.shape,dtype=np.float64)
        total = 0.0
        for i in range(35):       #current cell
            for j in range(35):
                for k in range(4):
                    if G[i][j][k] < 0.01:
                        continue
                    base = G_cen[i][j][k]
                    for l in range(35):
                        for m in range(35):
                        for n in range(4):
                            target = G_cen[l][m][n]
                            ptrans = math.sqrt((target.y-base.y)**2+(target.x-base.x)**2)
                            line_angle = math.atan2(target.y-base.y,target.x-base.x)                
                            if line_angle<0:
                                line_angle = 360 + line_angle
                            prot1 = get_rot(line_angle,base.z) #rot1
                            prot2 = get_rot(target.z,line_angle) #rot2
                            err_r1 = prot1 - rot1 
                            err_t = ptrans - trans
                            err_r2 = prot2 - rot2
                            #applying gaussian pmf    
                            P_rot1 = gaussian_rot*math.exp(-(err_r1**2)/(2*rot_var_noise**2))
                            P_trans = gaussian_trans*math.exp(-(err_t**2)/(2*trans_var_noise**2))   
                            P_rot2 = gaussian_rot*math.exp(-(err_r2**2)/(2*rot_var_noise**2))   
                            G_new[l][m][n] += G[i][j][k]*P_rot1*P_trans*P_rot2
                            total = total + G_new[l][m][n]
        G_new = G_new/total
        #print"--------Prediction done----------"
        G = G_new

    def update(range_tag,bearing,tagNum):
        global G,G_cen
        total1 = 0.0
        obs_matrix = np.zeros((35,35,4))
        for i in range(35):
        for j in range(35):
            for k in range(4):
                tag_position = tag[tagNum]
                p_range_tag = math.sqrt((tag_position.y - G_cen[i][j][k].y)^2 + (tag_position.y - G_cen[i][j][k].y)^2) #Range
                landmark_pose = math.atan2((tag_position.y - G_cen[i][j][k].y),(tag_position.x-G_cen[i][j][k].x)) #Bearing
                p_bearing = get_rot(G_cen[i][j][k].z,landmark_pose)
                err_range = p_range_tag - range_tag
                err_bearing = p_bearing - bearing
                P_range = gaussian_trans*math.exp(-(err_range**2)/(2*trans_var_noise**2))
                P_bearing = gaussian_rot*math.exp(-(err_bearing**2)/(2*rot_var_noise**2))
                #print P_range,P_bearing
                obs_matrix[i][j][k] = G[i][j][k]*P_range*P_bearing
                total1 = total1 + obs_matrix[i][j][k]
        obs_matrix = obs_matrix/total1
        print ("-----------Update Done------------")
        G = obs_matrix
        (u,v,w) = np.unravel_index(G.argmax(),G.shape)
        #print u,v,w
        w = w/90
        points = [3]
        points = [G_cen[u][v][w].x,G_cen[u][v][w].y,G_cen[u][v][w].z]
        mplot.plot(points[0],points[1],'ro')
        mplot.draw()
        mplot.pause(0.1)  
        pub_markers(u,v,w)  

class tags:

    def tags():
        global tag
        tag[0] = class_gcnts(125,525,0)
        tag[1] = class_gcnts(125,325,0)
        tag[2] = class_gcnts(125,125,0)
        tag[3] = class_gcnts(425,125,0)
        tag[4] = class_gcnts(425,325,0)
        tag[5] = class_gcnts(425,525,0)
        #print "--------tags initialized---------"

    def tag_poses():
        X=[1.25,1.25,1.25,4.25,4.25,4.25]
        Y=[5.25,3.25,1.25,1.25,3.25,5.25]
        X1=[125,125,125,425,425,425]
        Y1=[525,325,125,125,325,525]
        mplot.plot(X1,Y1,'g*')
        mplot.draw()
        #mplot.pause(0.1)
        size = len(X)
        tags_rviz(X,Y,size)

class visualization:

    def tags_rviz(X,Y,size):
        t_marker=[]
        for w in range(size):
            t_marker.append([])
            t_marker[w]=Marker()
            t_marker[w].pose.position.x = X[w]
            t_marker[w].pose.position.y = Y[w]
            t_marker[w].pose.position.z = 0
            t_marker[w].pose.orientation.x = 0.0
            t_marker[w].pose.orientation.y = 1.0
            t_marker[w].pose.orientation.z = 0.0
            t_marker[w].pose.orientation.w = 1.0
            t_marker[w].id=w
            t_marker[w].header.frame_id = 'map'
            t_marker[w].type=Marker.CUBE
            t_marker[w].ns = 'marker'
            t_marker[w].action= Marker.ADD
            t_marker[w].scale.x = 0.25
            t_marker[w].scale.y = 0.25
            t_marker[w].scale.z = 0.25
            t_marker[w].color.r = 0.0
            t_marker[w].color.g = 1.0
            t_marker[w].color.b = 0.0
            t_marker[w].color.a = 1.0
            t_marker[w].lifetime = rospy.Duration()
            marker_p.publish(t_marker[w])
            #print "------tags published-------"

if __name__ == '__main__':
    try:
        rospy.init_node('localization', anonymous=True)
        init_grid()
        tags()
        localize().run()
    except rospy.ROSInterruptException:
	pass
