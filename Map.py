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

    Map=255*np.ones((150*resolution,250*resolution,3))


    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):

            #for polygon right

            c1=clearance/math.sin(math.atan(-0.54)-math.pi/2)
            c2=clearance/math.sin(math.atan(0.60)-math.pi/2)
            c3=clearance/math.sin(math.atan(-0.18)-math.pi/2)

            if(i-135*resolution-clearance<=0 and j+0.54*i-245.97*resolution+c1<=0 and j-0.60*i-133.68*resolution+c2<=0 and j+0.18*i-181.05*resolution-c3>=0):
                Map[i,j]=[255,0,0]

            if(i-135*resolution-clearance<=0 and i-135*resolution>=0 and j+0.54*i-245.97*resolution+c1<=0 and j+0.54*i-245.97*resolution>=0 and math.pow(i-135*resolution,2)+math.pow(j-173*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j-0.60*i-133.68*resolution>=0 and j-0.60*i-133.68*resolution+c2<=0 and j+0.54*i-245.97*resolution+c1<=0 and j+0.54*i-245.97*resolution>=0 and math.pow(i-150*resolution+52*resolution,2)+math.pow(j-193*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j-0.60*i-133.68*resolution>=0 and j-0.60*i-133.68*resolution+c2<=0 and j+0.18*i-181.05*resolution-c3>=0 and j+0.18*i-181.05*resolution<=0 and math.pow(i-150*resolution+90*resolution,2)+math.pow(j-170*resolution,2)-math.pow(clearance,2)>=0):
                Map[i,j]=[255,255,255]


            c4=clearance/math.sin(math.atan(-0.18)-math.pi/2)
            c5=clearance/math.sin(math.atan(9.5)-math.pi/2)
            c6=clearance/math.sin(math.atan(0.6)-math.pi/2)

            #for polygon left
            if(i-135*resolution-clearance<=0 and j+0.18*i-181.05*resolution+c4<=0 and j-9.5*i+768.0*resolution+c5<=0 and j-0.6*i-67.68*resolution-c6>=0):
                Map[i,j]=[255,0,0]

            if(i-135*resolution-clearance<=0 and i-135*resolution>=0 and j-0.6*i-67.68*resolution-c6>=0 and j-0.6*i-67.68*resolution<=0 and math.pow(i-135*resolution,2)+math.pow(j-150*resolution,2)-math.pow(clearance,2)>=0):
                Map[i,j]=[255,255,255]

            if(j-9.5*i+768.0*resolution+c5<=0 and j-9.5*i+768.0*resolution>=0 and j-0.6*i-67.68*resolution-c6>=0 and j-0.6*i-67.68*resolution<=0 and math.pow(i-150*resolution+56*resolution,2)+math.pow(j-125*resolution,2)-math.pow(clearance,2)>=0):
                Map[i,j]=[255,255,255]

            #for rectrangle
            if(j>=50*resolution-clearance and j<=100*resolution+clearance and i>=37.5*resolution-clearance and i<=37.5*resolution+45*resolution+clearance):
                Map[i,j]=[255,0,0]

            if(j>=50*resolution-clearance and j<=50*resolution and i>=37.5*resolution-clearance and i<=37.5*resolution and math.pow(i-37.5*resolution,2)+math.pow(j-50*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j>=50*resolution-clearance and j<=50*resolution and i<=37.5*resolution+45*resolution+clearance and i>=37.5*resolution+45*resolution and math.pow(i-37.5*resolution-45*resolution,2)+math.pow(j-50*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j<=100*resolution+clearance and j>=100*resolution and i<=37.5*resolution+45*resolution+clearance and i>=37.5*resolution+45*resolution and math.pow(i-37.5*resolution-45*resolution,2)+math.pow(j-100*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            if(j<=100*resolution+clearance and j>=100*resolution and i>=37.5*resolution-clearance and i<=37.5*resolution and math.pow(i-37.5*resolution,2)+math.pow(j-100*resolution,2)-math.pow(clearance,2)>0):
                Map[i,j]=[255,255,255]

            #for circle
            if((math.pow(i-20*resolution,2)+math.pow(j-190*resolution,2))-math.pow(15*resolution+clearance,2)<=0):
                Map[i,j]=[255,0,0]

            #for ellipse
            if((math.pow(i-30*resolution,2)/math.pow(6*resolution+clearance,2))+math.pow(j-140*resolution,2)/math.pow(15*resolution+clearance,2)<=1):
                Map[i,j]=[255,0,0]




    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):
            if(j>50*resolution and j<100*resolution and i>37.5*resolution and i<37.5*resolution+45*resolution):
                Map[i,j]=[0,0,255]

            if((math.pow(i-20*resolution,2)+math.pow(j-190*resolution,2))<math.pow(15*resolution,2)):
                Map[i,j]=[0,0,255]

            if((math.pow(i-30*resolution,2)/math.pow(6*resolution,2))+math.pow(j-140*resolution,2)/math.pow(15*resolution,2)<1):
                Map[i,j]=[0,0,255]

            if(i-135*resolution<0 and j+0.54*i-245.97*resolution<0 and j-0.60*i-133.68*resolution<0 and j+0.18*i-181.05*resolution>0):
                Map[i,j]=[0,0,255]

            if(i-135*resolution<0 and j+0.18*i-181.05*resolution<0 and j-9.5*i+768.0*resolution<0 and j-0.6*i-67.68*resolution>0):
                Map[i,j]=[0,0,255]
    '''
    Y1=170
    X1=150-90
    Y2=163
    X2=150-52


    slope=(Y2-Y1)/(X2-X1)

    C=Y1-slope*X1

    print(slope)
    print(C)
    '''
    cv2.namedWindow('Map',cv2.WINDOW_NORMAL)
    cv2.imshow('Map',Map)
    cv2.waitKey()
    cv2.destroyAllWindows()
    #cv2.show()
    return Map

koko=Map(0,10,5)
