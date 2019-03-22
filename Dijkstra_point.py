import argparse
import numpy as np
import os, sys
from numpy import linalg as LA
import math
from PIL import Image
import random
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2

def Map(clearance):
    Map=255*np.ones((150,250,3)).astype(np.uint8)


    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):
            if(j>50-clearance and j<100+clearance and i>37-clearance and i<37+45+clearance):
                Map[i,j]=[0,0,255]

            if((math.pow(i-20,2)+math.pow(j-190,2))<math.pow(15+clearance,2)):
                Map[i,j]=[0,0,255]

            if((math.pow(i-30,2)/math.pow(6+clearance,2))+math.pow(j-140,2)/math.pow(15+clearance,2)<1):
                Map[i,j]=[0,0,255]

            if(i-135-clearance<0 and j+0.54*i-245.97<0 and j-0.60*i-133.68<0 and j+0.18*i-181.05>0):
                Map[i,j]=[0,0,255]

            if(i-135<0 and j+0.18*i-181.05<0 and j-9.5*i+768.0<0 and j-0.6*i-67.68>0):
                Map[i,j]=[0,0,255]
    return Map

def exp_right(Map,explorer,cost,x,y):
    if(y+1<250 and Map[x,y+1,0]==255):
        if(cost[x,y,0]+1<cost[x,y+1,0]):
            cost[x,y+1,0]=cost[x,y,0]+1
            cost[x,y+1,1]=x
            cost[x,y+1,2]=y
            explorer[x,y+1]=cost[x,y+1,0]
    return cost,explorer

def exp_left(Map,explorer,cost,x,y):
    if(y-1>-1 and Map[x,y-1,0]==255):
        if(cost[x,y,0]+1<cost[x,y-1,0]):
            cost[x,y-1,0]=cost[x,y,0]+1
            cost[x,y-1,1]=x
            cost[x,y-1,2]=y
            explorer[x,y-1]=cost[x,y-1,0]
    return cost,explorer

def exp_top(Map,explorer,cost,x,y):
    if(x-1>-1 and Map[x-1,y,0]==255):
        if(cost[x,y,0]+1<cost[x-1,y,0]):
            cost[x-1,y,0]=cost[x,y,0]+1
            cost[x-1,y,1]=x
            cost[x-1,y,2]=y
            explorer[x-1,y]=cost[x-1,y,0]
    return cost,explorer

def exp_bottom(Map,explorer,cost,x,y):
    if(x+1<150 and Map[x+1,y,0]==255):
        if(cost[x,y,0]+1<cost[x+1,y,0]):
            cost[x+1,y,0]=cost[x,y,0]+1
            cost[x+1,y,1]=x
            cost[x+1,y,2]=y
            explorer[x+1,y]=cost[x+1,y,0]
    return cost,explorer

def exp_top_right(Map,explorer,cost,x,y):
    if(x-1>-1 and y+1<250 and Map[x-1,y+1,0]==255):
        if(cost[x,y,0]+1.414<cost[x-1,y+1,0]):
            cost[x-1,y+1,0]=cost[x,y,0]+1.414
            cost[x-1,y+1,1]=x
            cost[x-1,y+1,2]=y
            explorer[x-1,y+1]=cost[x-1,y+1,0]
    return cost,explorer

def exp_top_left(Map,explorer,cost,x,y):
    if(x-1>-1 and y-1>-1 and Map[x-1,y-1,0]==255):
        if(cost[x,y,0]+1.414<cost[x-1,y-1,0]):
            cost[x-1,y-1,0]=cost[x,y,0]+1.414
            cost[x-1,y-1,1]=x
            cost[x-1,y-1,2]=y
            explorer[x-1,y-1]=cost[x-1,y-1,0]
    return cost,explorer

def exp_bottom_left(Map,explorer,cost,x,y):
    if(x+1<150 and y-1>-1 and Map[x+1,y-1,0]==255):
        if(cost[x,y,0]+1.414<cost[x+1,y-1,0]):
            cost[x+1,y-1,0]=cost[x,y,0]+1.414
            cost[x+1,y-1,1]=x
            cost[x+1,y-1,2]=y
            explorer[x+1,y-1]=cost[x+1,y-1,0]
    return cost,explorer

def exp_bottom_right(Map,explorer,cost,x,y):
    if(x+1<150 and y+1<250 and Map[x+1,y-1,0]==255):
        if(cost[x,y,0]+1.414<cost[x+1,y+1,0]):
            cost[x+1,y+1,0]=cost[x,y,0]+1.414
            cost[x+1,y+1,1]=x
            cost[x+1,y+1,2]=y
            explorer[x+1,y+1]=cost[x+1,y+1,0]
    return cost,explorer

def pointexplore(Map,explorer,cost,x,y):
    cost,explorer=exp_right(Map,explorer,cost,x,y)
    cost,explorer=exp_left(Map,explorer,cost,x,y)
    cost,explorer=exp_top(Map,explorer,cost,x,y)
    cost,explorer=exp_bottom(Map,explorer,cost,x,y)
    cost,explorer=exp_top_right(Map,explorer,cost,x,y)
    cost,explorer=exp_top_left(Map,explorer,cost,x,y)
    cost,explorer=exp_bottom_left(Map,explorer,cost,x,y)
    cost,explorer=exp_bottom_right(Map,explorer,cost,x,y)
    explorer[x,y]=5000
    Map[x,y]=[155,155,155]
    return cost,explorer,Map

def pathfinder(Startx,Starty,Endx,Endy,cost):
    path=np.mat([Endx,Endy])
    A=Endx
    B=Endy
    while(A!=Startx or B!=Starty):
        A_new=int(cost[A,B,1])
        B_new=int(cost[A,B,2])
        path=np.append([path],[A_new,B_new])
        A=A_new
        B=B_new
    length=int(path.shape[0]/2)
    new_path=np.reshape(path,(length,2))
    #print(new_path)
    return new_path

def draw(Map,path):
    for i in range(path.shape[0]):
        Map[path[path.shape[0]-1-i,0],path[path.shape[0]-1-i,1]]=[255,32,160]
        append_img=Map.copy()
        #print(i)
        img_array.append(append_img)
    return Map

def video(img_array):
    size=(250,150)
    video=cv2.VideoWriter('video1.avi',cv2.VideoWriter_fourcc(*'DIVX'),50.0,size)
    for i in range(len(img_array)):
        #print(len(img_array))
        #print(np.shape(img_array[i]))
        video.write(img_array[i])
    video.release()

#Point_free_space_old=Map(10)
Point_free_space=Map(0)
explorer=10000*np.array(np.ones((150,250)),dtype=float)
cost=10000*np.array(np.ones((150,250,3)),dtype=float)
Startx=1
Starty=1
explorer[Startx,Starty]=0
cost[Startx,Starty]=0
Endx=149
Endy=249
error=0
if (Point_free_space[Startx,Starty,0]==0):
    print('Start point is in obstacle')
    error=1
if (Point_free_space[Endx,Endy,0]==0):
    print('End point is in obstacle')
    error=1

img_array=[]
count=0
while(np.min(explorer)<5000 and error==0):
    least=np.argmin(explorer)
    exp_x=int(least/250)
    exp_y=least%250
    print(exp_x)
    print(exp_y)
    cost,explorer,Point_free_space=pointexplore(Point_free_space,explorer,cost,exp_x,exp_y)
    invideo=Point_free_space.copy()
    img_array.append(invideo)
    #count += 1
    #cv2.imwrite('%d.jpg' %count,Point_free_space)


if(error==0):
    Path=pathfinder(Startx,Starty,Endx,Endy,cost)
    print(Path)
    Pathmap=draw(Point_free_space,Path)

else:
    Pathmap=Point_free_space

video(img_array)
