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

def Map(clearance,radius,resolution):
    clearance=(clearance+radius)*resolution
    Map=255*np.ones((int(150*resolution),int(250*resolution),3)).astype(np.uint8)


    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):

            #for polygon right

            c1=clearance/math.sin(math.atan(-0.54)-math.pi/2)
            c2=clearance/math.sin(math.atan(0.60)-math.pi/2)
            c3=clearance/math.sin(math.atan(-0.18)-math.pi/2)

            if(i-135*resolution-clearance<0 and j+0.54*i-245.97*resolution+c1<0 and j-0.60*i-133.68*resolution+c2<0 and j+0.18*i-181.05*resolution-c3>0):
                Map[i,j]=[0,255,0]

            if(i-135*resolution-clearance<0 and i-135*resolution-0.6*clearance>=0 and j+0.54*i-245.97*resolution+c1<=0 and j+0.54*i-245.97*resolution-0.6*clearance>=0 and math.pow(i-135*resolution,2)+math.pow(j-173*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j-0.60*i-133.68*resolution-0.6*clearance>=0 and j-0.60*i-133.68*resolution+c2<=0 and j+0.54*i-245.97*resolution+c1<=0 and j+0.54*i-245.97*resolution-0.6*clearance>=0 and math.pow(i-150*resolution+52*resolution,2)+math.pow(j-193*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j-0.60*i-(133.68)*resolution+0.6*clearance>=0 and j-0.60*i-(133.68)*resolution+c2<=0 and j+0.18*i-(181.05)*resolution-c3>=0 and j+0.18*i-(181.05)*resolution-0.6*clearance<=0 and math.pow(i-150*resolution+90*resolution,2)+math.pow(j-170*resolution,2)-math.pow(clearance,2)>=0 ):
                Map[i,j]=[255,255,255]


            c4=clearance/math.sin(math.atan(-0.18)-math.pi/2)
            c5=clearance/math.sin(math.atan(9.5)-math.pi/2)
            c6=clearance/math.sin(math.atan(0.6)-math.pi/2)

            #for polygon left
            if(i-135*resolution-clearance<0 and j+0.18*i-181.05*resolution+c4<0 and j-9.5*i+768.0*resolution+c5<0 and j-0.6*i-67.68*resolution-c6>0):
                Map[i,j]=[0,255,0]

            if(i-135*resolution-clearance-0.2*clearance<0 and i-(135)*resolution-0.5*clearance>=0 and j-0.6*i-67.68*resolution-c6+0.2*clearance>=0 and j-0.6*i-(67.68)*resolution+0.3*clearance<=0 and math.pow(i-135*resolution,2)+math.pow(j-150*resolution,2)-math.pow(clearance,2)>=0):
                Map[i,j]=[255,255,255]

            if(j-9.5*i+(768.0)*resolution+c5<=0 and j-9.5*i+(768.0)*resolution+5*clearance>=0 and j-0.6*i-(67.68)*resolution+0.6*clearance-c6>=0 and j-0.6*i-(67.68)*resolution-0.6*clearance<=0 and math.pow(i-150*resolution+56*resolution,2)+math.pow(j-125*resolution,2)-math.pow(clearance,2)>=0):
                Map[i,j]=[255,255,255]

            #for rectrangle
            if(j>50*resolution-clearance and j<100*resolution+clearance and i>37.5*resolution-clearance and i<37.5*resolution+45*resolution+clearance):
                Map[i,j]=[0,255,0]

            if(j>=50*resolution-clearance and j<=50*resolution and i>=37.5*resolution-clearance and i<=37.5*resolution and math.pow(i-37.5*resolution,2)+math.pow(j-50*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j>=50*resolution-clearance and j<=50*resolution and i<=37.5*resolution+45*resolution+clearance and i>=37.5*resolution+45*resolution and math.pow(i-37.5*resolution-45*resolution,2)+math.pow(j-50*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j<=100*resolution+clearance and j>=100*resolution and i<=37.5*resolution+45*resolution+clearance and i>=37.5*resolution+45*resolution and math.pow(i-37.5*resolution-45*resolution,2)+math.pow(j-100*resolution,2)-math.pow(clearance,2)>0 and radius<14 ):
                Map[i,j]=[255,255,255]

            if(j<=100*resolution+clearance and j>=100*resolution and i>=37.5*resolution-clearance and i<=37.5*resolution and math.pow(i-37.5*resolution,2)+math.pow(j-100*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            #for circle
            if((math.pow(i-20*resolution,2)+math.pow(j-190*resolution,2))-math.pow(15*resolution+clearance,2)<0):
                Map[i,j]=[0,255,0]

            #for ellipse
            if((math.pow(i-30*resolution,2)/math.pow(6*resolution+clearance,2))+math.pow(j-140*resolution,2)/math.pow(15*resolution+clearance,2)<1):
                Map[i,j]=[0,255,0]

    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):
            if(j>50*resolution and j<100*resolution and i>37.5*resolution and i<37.5*resolution+45*resolution):
                Map[i,j]=[0,0,150]

            if((math.pow(i-20*resolution,2)+math.pow(j-190*resolution,2))<math.pow(15*resolution,2)):
                Map[i,j]=[0,0,255]

            if((math.pow(i-30*resolution,2)/math.pow(6*resolution,2))+math.pow(j-140*resolution,2)/math.pow(15*resolution,2)<1):
                Map[i,j]=[0,0,255]

            if(i-135*resolution<0 and j+0.54*i-245.97*resolution<0 and j-0.60*i-133.68*resolution<0 and j+0.18*i-181.05*resolution>0):
                Map[i,j]=[0,0,255]

            if(i-135*resolution<0 and j+0.18*i-181.05*resolution<0 and j-9.5*i+768.0*resolution<0 and j-0.6*i-67.68*resolution>0):
                Map[i,j]=[0,0,255]
    return Map

def exp_right(Map,explorer,cost,x,y):
    limit=250*resolution
    if(y+1<limit and Map[x,y+1,0]==255):
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
    limit=int(150*resolution)
    if(x+1<limit and Map[x+1,y,0]==255):
        if(cost[x,y,0]+1<cost[x+1,y,0]):
            cost[x+1,y,0]=cost[x,y,0]+1
            cost[x+1,y,1]=x
            cost[x+1,y,2]=y
            explorer[x+1,y]=cost[x+1,y,0]
    return cost,explorer

def exp_top_right(Map,explorer,cost,x,y):
    if(x-1>-1 and y+1<int(250*resolution) and Map[x-1,y+1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x-1,y+1,0]):
            cost[x-1,y+1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x-1,y+1,1]=x
            cost[x-1,y+1,2]=y
            explorer[x-1,y+1]=cost[x-1,y+1,0]
    return cost,explorer

def exp_top_left(Map,explorer,cost,x,y):
    if(x-1>-1 and y-1>-1 and Map[x-1,y-1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x-1,y-1,0]):
            cost[x-1,y-1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x-1,y-1,1]=x
            cost[x-1,y-1,2]=y
            explorer[x-1,y-1]=cost[x-1,y-1,0]
    return cost,explorer

def exp_bottom_left(Map,explorer,cost,x,y):
    if(x+1<int(150*resolution) and y-1>-1 and Map[x+1,y-1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x+1,y-1,0]):
            cost[x+1,y-1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x+1,y-1,1]=x
            cost[x+1,y-1,2]=y
            explorer[x+1,y-1]=cost[x+1,y-1,0]
    return cost,explorer

def exp_bottom_right(Map,explorer,cost,x,y):
    if(x+1<int(150*resolution) and y+1<int(250*resolution) and Map[x+1,y+1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x+1,y+1,0]):
            cost[x+1,y+1,0]=cost[x,y,0]+np.sqrt(2)
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
    explorer[x,y]=500000
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
        invideo=Map.copy()
        cv2.imshow('Map',invideo)
        cv2.waitKey(50)
    return Map

def video(img_array):
    size=(int(250*resolution),int(150*resolution))
    video=cv2.VideoWriter('video1.avi',cv2.VideoWriter_fourcc(*'DIVX'),50.0,size)
    for i in range(len(img_array)):
        video.write(img_array[i])
    video.release()

#Point_free_space_old=Map(10)
clearance=int(input('value of Clearance \n'))
radius=int(input('value of Radius \n'))
resolution=float(input('value of Resolution \n'))

Point_free_space=Map(clearance,radius,resolution)
print("Map is ready")
explorer=1000000*np.array(np.ones((int(150*resolution),int(250*resolution))),dtype=float)
cost=1000000*np.array(np.ones((int(150*resolution),int(250*resolution),3)),dtype=float)
distance=np.array(np.ones((int(150*resolution),int(250*resolution))),dtype=float)
print("Import co-ordinates of x between 0 &",int(150*resolution)-1 )
print("Import co-ordinates of y between 0 &",int(250*resolution)-1 )

error=0
Startx=int(input('value of Start x \n'))
Starty=int(input('value of Start y \n'))
Endx=int(input('value of Goal x \n'))
Endy=int(input('value of Goal y \n'))


if (Startx<0 or Startx>(int(150*resolution)-1) or Starty<0 or Starty>(int(250*resolution)-1)):
    print('Start point out of bound')
    error=1

if (Endx<0 or Endx>int(150*resolution)-1 or Endy<0 or Endy>int(250*resolution)-1):
    print('End point out of bound')
    error=1

if (error==0):
    if (Point_free_space[Startx,Starty,0]==0):
        print('Start point is in obstacle')
        error=1
    if (Point_free_space[Endx,Endy,0]==0):
        print('End point is in obstacle')
        error=1
    explorer[Startx,Starty]=0
    cost[Startx,Starty]=0
    Point_free_space[Startx,Starty]=[255,100,20]
    Point_free_space[Endx,Endy]=[255,100,20]

#------------------------------------------------------------
for i in range(distance.shape[0]):
    for j in range(distance.shape[1]):
        if(np.abs(i-Endx)>np.abs(j-Endy)):
            distance[i,j]=np.abs(np.abs(i-Endx)-np.abs(j-Endy))*1 +np.abs(j-Endy)*np.sqrt(2)
        else:
            distance[i,j]=np.abs(np.abs(i-Endx)-np.abs(j-Endy))*1 +np.abs(i-Endx)*np.sqrt(2)


#------------------------------------------------------------
img_array=[]
count=0
#print('idher hu')
if (error==0):
    while(explorer[Endx,Endy]!=500000):
        least=np.argmin(explorer+distance)
        exp_x=int(least/(250*resolution))
        exp_y=int(least%(250*resolution))
        #print(exp_x)
        #print(exp_y)
        cost,explorer,Point_free_space=pointexplore(Point_free_space,explorer,cost,exp_x,exp_y)
        invideo=Point_free_space.copy()
        cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
        cv2.imshow('Map',invideo)
        cv2.waitKey(1)
        #img_array.append(invideo)
        count += 1
        #print(count)

    print(distance[Startx,Starty],'Minimum distance')
    print(cost[Endx,Endy,0],'Calculated Distance')
    Path=pathfinder(Startx,Starty,Endx,Endy,cost)
    #print(Path)
    Pathmap=draw(Point_free_space,Path)

else:
    Pathmap=Point_free_space
