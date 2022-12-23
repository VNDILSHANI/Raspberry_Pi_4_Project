#!/usr/bin/python
import cv2
import time
import numpy as np




#--------------GPIO------------------
import RPi.GPIO as GPIO  
       
#--right side
in1 = 17
in2 = 27
ena = 22

#--left side
in3 = 13
in4 = 19
enb = 26

GPIO.setmode(GPIO.BCM)

#-------------- pin mode define-----------
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(ena,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)


#--------set frequency----------
p_right=GPIO.PWM(ena,100)
p_left=GPIO.PWM(enb,100)

#- set duty cyple------------
p_right.start(50)
p_left.start(50)

def forward():
    
    #--Right
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    #--Left
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    print("forward")


#--------------------------------------------------------
# use zero with pi camera
cols = frameWidth = 640
rows= frameHeight = 144
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)  
cap.set(4, frameHeight)

setpoint = cols/2


while True:
    
    ret , frame = cap.read()
    
    if (ret==1):
   

     
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red color
        low_red = np.array([0, 0,0])
        high_red = np.array([179, 255, 87])
        
        mask = cv2.inRange(hsv_frame, low_red, high_red)
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if (len(contours)>0):
            
            for cnt in contours:
                
                (x, y, w, h) = cv2.boundingRect(cnt)
                x_road_centre = x+w/2
                
                distance_from_centre = cols/2 - (x+w/2)
                
                # draw a rectangle in the contour area
                cv2.rectangle(frame,(x,y*2),(x+w,y+h),(0,0,255),3)
                
                # distance_from_centre line
                cv2.line(frame, (int(cols/2), int(y+h/2)), (int(x+w/2),int(y+h/2)), (0, 255, 0), 2)
                
                    
                #turn left
                if (x+w/2 <= cols/2 ):
	
       		    p_left.start(50)
                    p_right.start(100)
                    forward()
                    print("turn_left")
           
           
                #turn right
                if (x+w/2 > cols/2):
                    
		    p_left.start(100)
                    p_right.start(150)
                    forward()
                    print("turn_right")
                
                break
            
            
        # draw centre line
        cv2.line(frame, (int(cols/2), 0), (int(cols/2), rows), (0, 255, 0), 5)
  
        
        cv2.imshow("Frame",frame)
        cv2.imshow("mask",mask)
        #plt.show()
        key = cv2.waitKey(1)
        if key  == 27:
            break
            
        
cap.release()
cv2.destroyAllWindows()
    


     
