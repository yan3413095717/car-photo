import cv2
import sys
from __future__ import division
import serial
import numpy as np
import logging as log
import datetime as dt
from time import sleep
import RPi.GPIO as GPIO
import time
import os
import pygame
import multiprocessing as mp
from multiprocessing import Pool,current_process
os.putenv( 'SDL_FBDEV', '/dev/fb1' )
def car_control(x,y):
    if x > 340:    #判断标志flag
        ser.write("M000\r\n")//串口发送X
        
    elif x < 300 and x > 0:    #判断标志flag
        ser.write("M180\r\n")//串口发送X
       
    
    elif y > 120:
        ser.write("M090\r\n")//串口发送A
        
        
    elif (y<80 and y>0):
        ser.write("M360\r\n")
        
    elif (x<=340 and x>=300 and y>=80 and y<=120):
        ser.write("M999\r\n")


#video_capture = cv2.VideoCapture(0)
#classifier = cv2.CascadeClassifier( '/home/pi/opencv-3.4.1/data/lbpcascades/haarcascade_frontalface_default.xml') 
#color = (0, 255, 0) # 定义绘制颜色 
# 调用识别人脸 
#faceRects = classifier.detectMultiScale( gray, scaleFactor=1.2, minNeighbors=3, minSize=(32, 32)) 
### Setup #####################################################################

resX = 640
resY = 480


# The face cascade file to be used

faceCascade = cv2.CascadeClassifier('/home/pi/t2/haarcascade_frontalface_default.xml')
face_color = (192,220,240)   
strokeWeight = 2 
t_start = time.time()
fps = 0

### Helper Functions ##########################################################

def get_faces( img ):

    gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    faces = faceCascade.detectMultiScale(gray, 1.3, 5)

    return faces, img, gray

def draw_frame( faces, img, gray):

    global xdeg
    global ydeg
    global fps
    global time_t

    for x, y, w, h in faces:

        cv2.rectangle( img, ( x, y ),( x + w, y + h ), face_color, strokeWeight)       
        cv2.putText(img, "Face No." + str( len( faces ) ), ( x, y ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ( 0, 0, 255 ), 2 )
        face_gray=gray[y:y+h,x:x+w]
        car_control(x,y)
        

 

    fps = fps + 1
    sfps = fps / (time.time() - t_start)
    cv2.putText(img, "FPS : " + str( int( sfps ) ), ( 10, 10 ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ( 0, 0, 255 ), 2 )

    cv2.imshow( "Frame", img )


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyAMA0', 9600)
if ser.isOpen == False:
    ser.open()

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,resX)  
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,resY) 

    pool = mp.Pool( processes=4 )

    read, img = camera.read()
    pr1 = pool.apply_async( get_faces, [ img ] )   
    read, img = camera.read()
    pr2 = pool.apply_async( get_faces, [ img ] )  
    read, img = camera.read() 
    pr3 = pool.apply_async( get_faces, [ img ] )   
    read, img = camera.read()
    pr4 = pool.apply_async( get_faces, [ img ] )   

    fcount = 1

    while (True):
        read, img = camera.read()

        if   fcount == 1:
            pr1 = pool.apply_async( get_faces, [ img ] )
            faces, img, gray=pr2.get()
            draw_frame( faces, img, gray )

        elif fcount == 2:
            pr2 = pool.apply_async( get_faces, [ img ] )
            faces, img, gray=pr3.get()
            draw_frame( faces, img, gray )

        elif fcount == 3:
            pr3 = pool.apply_async( get_faces, [ img ] )
            faces, img, gray=pr4.get()
            draw_frame( faces, img, gray )

        elif fcount == 4:
            pr4 = pool.apply_async( get_faces, [ img ] )
            faces, img, gray=pr1.get()
            draw_frame( faces, img, gray )
            fcount = 0


        fcount += 1

        if cv2.waitKey(1000 // 12) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
    


            
            

