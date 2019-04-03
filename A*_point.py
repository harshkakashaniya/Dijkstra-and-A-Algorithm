'''
/**
* @file A*_point.py
* @author Kakashaniya Harsh
* @date 22 Mar 2019
* @copyright 2019 Kakashaniya Harsh
* @brief Implementation of A* algorithm.
*/
'''

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
'''
/**
* @brief function is used to generate Map
* @param [in] clearance,radius,resolution
* @return Map
* @details uses Half planes to generate map.
*/
'''

def Slope(X1,Y1,X2,Y2):
    slope=(Y2-Y1)/(X2-X1)
    C=Y1-slope*X1
    return slope,C
'''
/**
* @brief function is used to generate rectrangle
* @param [in] Map,clearance,radius,resolution,x ,y
* @return Map
* @details uses Half planes to generate map.
*/
'''
def rectrangle(Map,resolution,radius,clearance,i,j):
    Xrecmin=36.5*resolution
    Xrecmax=36.5*resolution+45*resolution
    Yrecmin=49*resolution
    Yrecmax=99*resolution

    if(j>Yrecmin-clearance and j<Yrecmax+clearance and i>Xrecmin-clearance and i<Xrecmax+clearance):
        Map[i,j]=[0,255,0]

    if(resolution>0.5 and (clearance/resolution)<10):
        if(j>=Yrecmin-clearance and j<=Yrecmin and i>=Xrecmin-clearance and i<=Xrecmin and math.pow(i-Xrecmin,2)+math.pow(j-Yrecmin,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

        if(j>=Yrecmin-clearance and j<=Yrecmin and i<=Xrecmax+clearance and i>=Xrecmax and math.pow(i-Xrecmin-45*resolution,2)+math.pow(j-Yrecmin,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

        if(j<=Yrecmax+clearance and j>=Yrecmax and i<=Xrecmax+clearance and i>=Xrecmax and math.pow(i-Xrecmin-45*resolution,2)+math.pow(j-Yrecmax,2)-math.pow(clearance,2)>=0 and radius<14 ):
            Map[i,j]=[255,255,255]

        if(j<=Yrecmax+clearance and j>=Yrecmax and i>=Xrecmin-clearance and i<=Xrecmin and math.pow(i-Xrecmin,2)+math.pow(j-Yrecmax,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

    if(j>Yrecmin and j<Yrecmax and i>Xrecmin and i<Xrecmax):
        Map[i,j]=[0,0,150]

    return Map
'''
/**
* @brief function is used to generate circle
* @param [in] Map,clearance,radius,resolution,x ,y
* @return Map
* @details uses Semi algebric model to generate map.
*/
'''
def circle(Map,resolution,radius,clearance,i,j):
    X=np.int(19*resolution)
    Y=np.int(189*resolution)
    rad=np.int(15*resolution)
    if((math.pow(i-X,2)+math.pow(j-Y,2))-math.pow(rad+clearance,2)<=0):
        Map[i,j]=[0,255,0]

    if((math.pow(i-X,2)+math.pow(j-Y,2))-math.pow(rad,2)<=0):
        Map[i,j]=[0,0,150]
    return Map
'''
/**
* @brief function is used to generate ellipse
* @param [in] Map,clearance,radius,resolution,x ,y
* @return Map
* @details uses Semi algebric model to generate map.
*/
'''
def ellipse(Map,resolution,radius,clearance,i,j):
    X=np.int(29*resolution)
    Y=np.int(139*resolution)
    minima=math.ceil(6*resolution)
    maxima=math.ceil(15*resolution)

    if((math.pow(i-X,2)/math.pow(minima+clearance,2))+(math.pow(j-Y,2)/math.pow(maxima+clearance,2))-1<=0):
        Map[i,j]=[0,255,0]

    if((math.pow(i-X,2)/math.pow(minima,2))+(math.pow(j-Y,2)/math.pow(maxima,2))-1<=0):
        Map[i,j]=[0,0,150]
    return Map
'''
/**
* @brief function is used to generate polygon
* @param [in] Map,clearance,radius,resolution,x ,y
* @return Map
* @details uses Half planes to generate map.
*/
'''
def irregular(Map,resolution,radius,clearance,i,j):
    X1,Y1=int(134*resolution),int(resolution*172)
    X2,Y2=int((149-52)*resolution),int(192*resolution)
    X3,Y3=int((149-90)*resolution),int(169*resolution)
    X4,Y4=int((149-52)*resolution),int(162*resolution)
    X5,Y5=int((149-56)*resolution),int(125*resolution)
    X6,Y6=int((149-15)*resolution),int(149*resolution)

    m1,con1=Slope(X1,Y1,X2,Y2)
    m2,con2=Slope(X2,Y2,X3,Y3)
    m3,con3=Slope(X3,Y3,X4,Y4)
    if((X4-X5)==0):
        m4,con4=Slope((149-52)*resolution,Y4,(149-56)*resolution,Y5)
    else:
        m4,con4=Slope(X4,Y4,X5,Y5)
    m5,con5=Slope(X5,Y5,X6,Y6)
    #print(m5,con3)

    c1=clearance/math.sin(math.atan(m1)-math.pi/2)
    c2=clearance/math.sin(math.atan(m2)-math.pi/2)
    c3=clearance/math.sin(math.atan(m3)-math.pi/2)

    if(i-X1-clearance<=0 and j-m1*i-con1+c1<=0 and j-m2*i-con2+c2<=0 and j-m3*i-con3-c3>=0):
        Map[i,j]=[0,255,0]
    if(resolution>0.5):
        if(i-X1-clearance<=0 and i-X1-0.6*clearance>=0 and j-m1*i-con1+c1<=0 and j-m1*i-con1-0.6*clearance>=0 and math.pow(i-X1,2)+math.pow(j-Y1,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

        if(j-m2*i-con2-0.6*clearance>=0 and j-m2*i-con2+c2<=0 and j-m1*i-con1+c1<=0 and j+-m1*i-con1-0.6*clearance>=0 and math.pow(i-X2,2)+math.pow(j-Y2,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

        if(j-m2*i-(con2)+0.6*clearance>=0 and j-m2*i-(con2)+c2<=0 and j-m3*i-(con3)-c3>=0 and j-m3*i-(con3)-0.6*clearance<=0 and math.pow(i-X3,2)+math.pow(j-Y3,2)-math.pow(clearance,2)>=0 ):
            Map[i,j]=[255,255,255]

    c4=clearance/math.sin(math.atan(m3)-math.pi/2)
    c5=clearance/math.sin(math.atan(m4)-math.pi/2)
    c6=clearance/math.sin(math.atan(m5)-math.pi/2)

    #for polygon left
    if(i-X1-clearance<=0 and j-m3*i-con3+c4<=0 and j-m4*i-con4+c5<=0 and j-m5*i-con5-c6>=0):
        Map[i,j]=[0,255,0]
    if(resolution>0.5):
        if(i-X1-clearance-0.2*clearance<=0 and i-(X1)-0.5*clearance>=0 and j-m5*i-con5-c6+0.2*clearance>=0 and j-m5*i-(con5)+0.3*clearance<=0 and math.pow(i-X1,2)+math.pow(j-Y6,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

        if(j-m4*i+(-con4)+c5<=0 and j-m4*i+(-con4)+5*clearance>=0 and j-m5*i-(con5)+0.6*clearance-c6>=0 and j-m5*i-(con5)-0.6*clearance<=0 and math.pow(i-X5,2)+math.pow(j-Y5,2)-math.pow(clearance,2)>=0):
            Map[i,j]=[255,255,255]

    if(i-X1<=0 and j-m1*i-con1<=0 and j-m2*i-con2<=0 and j-m3*i-con3>=0):
        Map[i,j]=[0,0,150]

    if(i-X1<=0 and j-m3*i-con3<=0 and j-m4*i-con4<=0 and j-m5*i-con5>=0):
        Map[i,j]=[0,0,150]

    return Map
'''
/**
* @brief function is used to generate Map
* @param [in] clearance,radius,resolution
* @return Map
* @details uses Half planes to generate map.
*/
'''
def Map(clearance,radius,resolution):
    #resolution=np.round(1/resolution,1)
    clearance=(clearance+radius)*resolution
    Map=255*np.ones((int(150*resolution),int(250*resolution),3)).astype(np.uint8)
    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):
            Map=irregular(Map,resolution,radius,clearance,i,j)
            Map=circle(Map,resolution,radius,clearance,i,j)
            Map=ellipse(Map,resolution,radius,clearance,i,j)
            Map=rectrangle(Map,resolution,radius,clearance,i,j)
            if (resolution>0.20):
                if (j<np.int(radius*resolution) or j>np.int((250-radius)*resolution) or i<np.int(radius*resolution) or i>np.int((150-radius)*resolution)):
                    Map[i,j]=[0,255,0]
    #cv2.namedWindow('Map',cv2.WINDOW_NORMAL)
    #cv2.imshow('Map',Map)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    return Map
'''
/**
* @brief exp_right
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore right
*/
'''
def exp_right(Map,explorer,cost,x,y):
    limit=250*resolution
    if(y+1<limit and Map[x,y+1,0]==255):
        if(cost[x,y,0]+1<cost[x,y+1,0]):
            cost[x,y+1,0]=cost[x,y,0]+1
            cost[x,y+1,1]=x
            cost[x,y+1,2]=y
            explorer[x,y+1]=cost[x,y+1,0]
            Map[x,y+1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_left
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore left
*/
'''
def exp_left(Map,explorer,cost,x,y):
    if(y-1>-1 and Map[x,y-1,0]==255):
        if(cost[x,y,0]+1<cost[x,y-1,0]):
            cost[x,y-1,0]=cost[x,y,0]+1
            cost[x,y-1,1]=x
            cost[x,y-1,2]=y
            explorer[x,y-1]=cost[x,y-1,0]
            Map[x,y-1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_top
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore top
*/
'''
def exp_top(Map,explorer,cost,x,y):
    if(x-1>-1 and Map[x-1,y,0]==255):
        if(cost[x,y,0]+1<cost[x-1,y,0]):
            cost[x-1,y,0]=cost[x,y,0]+1
            cost[x-1,y,1]=x
            cost[x-1,y,2]=y
            explorer[x-1,y]=cost[x-1,y,0]
            Map[x-1,y]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_bottom
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore bottom
*/
'''
def exp_bottom(Map,explorer,cost,x,y):
    limit=int(150*resolution)
    if(x+1<limit and Map[x+1,y,0]==255):
        if(cost[x,y,0]+1<cost[x+1,y,0]):
            cost[x+1,y,0]=cost[x,y,0]+1
            cost[x+1,y,1]=x
            cost[x+1,y,2]=y
            explorer[x+1,y]=cost[x+1,y,0]
            Map[x+1,y]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_top_right
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore top_right
*/
'''
def exp_top_right(Map,explorer,cost,x,y):
    if(x-1>-1 and y+1<int(250*resolution) and Map[x-1,y+1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x-1,y+1,0]):
            cost[x-1,y+1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x-1,y+1,1]=x
            cost[x-1,y+1,2]=y
            explorer[x-1,y+1]=cost[x-1,y+1,0]
            Map[x-1,y+1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_top_left
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore top_left
*/
'''
def exp_top_left(Map,explorer,cost,x,y):
    if(x-1>-1 and y-1>-1 and Map[x-1,y-1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x-1,y-1,0]):
            cost[x-1,y-1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x-1,y-1,1]=x
            cost[x-1,y-1,2]=y
            explorer[x-1,y-1]=cost[x-1,y-1,0]
            Map[x-1,y-1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_bottom_left
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore top_left
*/
'''
def exp_bottom_left(Map,explorer,cost,x,y):
    if(x+1<int(150*resolution) and y-1>-1 and Map[x+1,y-1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x+1,y-1,0]):
            cost[x+1,y-1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x+1,y-1,1]=x
            cost[x+1,y-1,2]=y
            explorer[x+1,y-1]=cost[x+1,y-1,0]
            Map[x+1,y-1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief exp_bottom_right
* @param Map,explorer,cost,x,y
* @return cost,explorer
* @details To explore top_right
*/
'''
def exp_bottom_right(Map,explorer,cost,x,y):
    if(x+1<int(150*resolution) and y+1<int(250*resolution) and Map[x+1,y+1,0]==255):
        if(cost[x,y,0]+np.sqrt(2)<cost[x+1,y+1,0]):
            cost[x+1,y+1,0]=cost[x,y,0]+np.sqrt(2)
            cost[x+1,y+1,1]=x
            cost[x+1,y+1,2]=y
            explorer[x+1,y+1]=cost[x+1,y+1,0]
            Map[x+1,y+1]=[155,155,155]
            invideo=Map.copy()
            cv2.imshow('Map',invideo)
            cv2.waitKey(1)
            img_array.append(invideo)
    return cost,explorer
'''
/**
* @brief pointexplore
* @param Map,explorer,cost,x,y
* @return cost,explorer,Map
* @details This function is used to explore all sides around a node.
*/
'''
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
'''
/**
* @brief pathfinder
* @param Startx,Starty,Endx,Endy,cost
* @return new_path
* @details This function is used to back track optimal node.
'''
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
'''
/**
* @brief draw
* @param Map,path
* @return Map
* @details This function colours the path.
'''
def draw(Map,path):
    for i in range(path.shape[0]):
        Map[path[path.shape[0]-1-i,0],path[path.shape[0]-1-i,1]]=[255,32,160]
        invideo=Map.copy()
        cv2.imshow('Map',invideo)
        img_array.append(invideo)
        if(resolution<3):
            cv2.waitKey(int(10/resolution))
        else:
            cv2.waitKey(1)
    print('Output will stay for 5 seconds')
    print('Output is also saved in folder')
    cv2.imshow('Map',invideo)
    img_array.append(invideo)
    cv2.waitKey(5000)
    return Map
'''
/**
* @brief video
* @param img_array
*
* @details To publish video
'''
def video(img_array):
    size=(int(250*resolution),int(150*resolution))
    video=cv2.VideoWriter('video_A_point.avi',cv2.VideoWriter_fourcc(*'DIVX'),50.0,size)
    for i in range(len(img_array)):
        video.write(img_array[i])
    video.release()

# initialization matrix
error=0
clearance=0
radius=0
print("Input resolution amongest following values: 5 , 2 , 1 , 0.5 , 0.25 , 0.2" )
reso=float(input('value of Resolution \n'))
resolution=float(round(1/reso,1))
print("Map is getting ready")
if (reso!=5 and reso!=2 and reso!=1 and reso!=0.5 and reso!=0.25 and reso!=0.2):
    error=1
    print('Resolution wrong!!!!')
    print('Please enter it as specified above')


if (error==0):
    explorer=1000000*np.array(np.ones((int(150*resolution),int(250*resolution))),dtype=float)
    cost=1000000*np.array(np.ones((int(150*resolution),int(250*resolution),3)),dtype=float)
    distance=np.array(np.ones((int(150*resolution),int(250*resolution))),dtype=float)
    Point_free_space=Map(clearance,radius,resolution)
    print("Map is ready")

    # Initiating window
    cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
    cv2.imshow('Map',Point_free_space)
    cv2.waitKey(2000)
    # input user variables
    print("Input co-ordinates has origin at bottom left" )
    print("Input co-ordinates of x between 0 &",int(250*resolution)-1 )
    print("Input co-ordinates of y between 0 &",int(150*resolution)-1 )
    error=0
    Startx_input=int(input('Value of Start x \n'))
    Starty_input=int(input('Value of Start y \n'))
    Endx_input=int(input('Value of Goal x \n'))
    Endy_input=int(input('Value of Goal y \n'))
    Starty=Startx_input
    Startx=int(150*resolution)-1-Starty_input
    Endy=Endx_input
    Endx=int(150*resolution)-1-Endy_input
    # Error conditions
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
        Path_map=Point_free_space.copy()

    #-------------------------------------------------------------------------------
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            if(np.abs(i-Endx)>np.abs(j-Endy)):
                #distance[i,j]=np.abs(np.abs(i-Endx)-np.abs(j-Endy))*1 +np.abs(j-Endy)*np.sqrt(2)
                #distance[i,j]=np.abs(np.abs(i-Endx)+np.abs(j-Endy))
                distance[i,j]=np.sqrt((i-Endx)**2+(j-Endy)**2)
            else:
                #distance[i,j]=np.abs(np.abs(i-Endx)-np.abs(j-Endy))*1 +np.abs(i-Endx)*np.sqrt(2)
                #distance[i,j]=np.abs(np.abs(i-Endx)+np.abs(j-Endy))
                distance[i,j]=np.sqrt((i-Endx)**2+(j-Endy)**2)
#-------------------------------------------------------------------------------

img_array=[]
count=0

# Main loop to explore
if (error==0):
    while(explorer[Endx,Endy]!=500000 and np.min(explorer)!=500000):
        #print(np.min(explore))
        least=np.argmin(explorer+distance)
        exp_x=int(least/(250*resolution))
        exp_y=int(least%(250*resolution))
        cost,explorer,Point_free_space=pointexplore(Point_free_space,explorer,cost,exp_x,exp_y)
        count += 1
        #print(count)


    if (cost[Endx,Endy,0]>500000):
        print('Not possible to reach goal because of Obstacles')
    else:
        print(cost[Endx,Endy,0],'Calculated Distance')
        #print(distance[Startx,Starty],'Minimum distance')
        Path=pathfinder(Startx,Starty,Endx,Endy,cost)
        Pathmap=draw(Path_map,Path)

#else:
    #Pathmap=Point_free_space

    video(img_array)
