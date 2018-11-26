#
# File: Contour_Detect_Drive
#
# Author: Jacob Gerecht and Austin Smith
#
# Created: 11-09-2018
#
# Purpose: This code locates the orange circle that makes up our beacon and indicates
# it with a bright green circle on the displayed image. The detected blob is also printed
# to the console to provide feedback information. The code also estimates the distance and angle
# the target is located at. The key change here is using contours to define the detected blob,
# then using a minimal enclosing circle to estimate the size. This is to deal with the issue
# of viewing the beacon like an elipse
# 
# Use: Ensure that the pi is running on the cv environment. Make sure the picamera
# is properly attached and installed with the pip install "picamra[array]" command.
# Make sure cv2.so file is in the same directory as the script
#

#Setup I2C libraries and LCD
import smbus
import Adafruit_CharLCD as LCD
import subprocess

#Setting up all relevent libraries
from picamera import PiCamera
from time import sleep
import numpy as np
import cv2

#setup I2C
bus = smbus.SMBus(1)
address = 0x04

#flags for beacon detection
beacon_flag = False
arduinoReady = False

#LCD Setup
##while(True):
##    try:
##        lcd = LCD.Adafruit_CharLCDPlate()
##        break
##    except IOError as e:
##        subprocess.call(['i2cdetect', '-y', '1'])
##        print("Error connecting to LCD, retrying")


#function for sending data to Arduino
def sendData(signal1, signal2, signal3):
    print("sending data: ", signal1, signal2, signal3)
    try:
        bus.write_i2c_block_data(address, 2, [signal1, signal2, signal3])
    except IOError as e:
        subprocess.call(['i2cdetect', '-y', '1'])
        print("Trying to send data again")
        sendData(signal1, signal2, signal3)
    return


#function for receiving data from Arduino
def ReceiveData():
    print("Receiving data")
    try:
        temp = bus.read_byte(address)
        print("readyflag variable from arduino =", temp)
        return temp
    except IOError as e:
        subprocess.call(['i2cdetect', '-y', '1'])
        print("reasking for data")
        ReceiveData()
    return

def LCDSend(message, var):
    print("Sending: ", message, " to LCD")
    print("Sending: ", var, " to LCD")
    try:
        #sending Message and Variable to LCD
        lcd.clear()
        lcd.message(message + " \n")
        lcd.message(str(var))
    except IOError as e:
        subprocess.call(['i2cdetect', '-y', '1'])
        LCDSend(message, var)
    return

def LCDSend1(message):
    print("Sending: ", message, " to LCD")
    try:
        #sending Message and Variable to LCD
        lcd.clear()
        lcd.message(message + " \n")
    except IOError as e:
        subprocess.call(['i2cdetect', '-y', '1'])
        LCDSend1(message)
    return

print("I2C ready")

#variables for sending to arduino
desiredDistance = 0
desiredAngle = 0

camera = PiCamera()
camera.resolution = (640, 640)

lower_g = np.array([55, 70, 60])
upper_g = np.array([80, 200, 155])

#1 is to the left, -1 is to the right, 0 is straight on
direction = 0;


while (1):

    #creates an empty numpy array to store the image in
    image = np.empty((640*640*3,), dtype=np.uint8)

    #captures the image and stores in numpy array
    camera.capture(image, 'bgr')

    #Organizes array into 320 X 320 X 3 arrangements for x, y, and brg placement
    image = image.reshape((640, 640, 3))
   
    #Converts to HSV color scheme
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #Green Filter
    green_final = cv2.inRange(hsv_img, lower_g, upper_g)

    #Removes noise from the color isolated images
    kernel_erode = np.ones((15,15),np.uint8)
    kernel_dilate = np.ones((10,10),np.uint8)

    green_final = cv2.morphologyEx(green_final, cv2.MORPH_OPEN, kernel_erode)

    green_final = cv2.morphologyEx(green_final, cv2.MORPH_DILATE, kernel_dilate)

    #locates the contours
    green_final, contours, hierarchy = cv2.findContours(green_final, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    #for each found contour, filter for both size and circularity
    beacons = []
    for contour in contours:
        #approximation for how cirular the contour is
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True),True)
        area = cv2.contourArea(contour)
        #print len(approx)
        if ((len(approx) > 4) and (len(approx) < 23) and (area > 30)):
            beacons.append(contour) #if it fits, add it to our list
            
    if(len(beacons) >= 1):
        beacon_flag = True
    else:
        beacon_flag = False

    if(beacon_flag == False):
        desiredAngle = 45
        print("beacon not found, angle set to 45")

    for beacon in beacons:
        #incribe an enclosing circle around the found beacon
        (x,y),radius = cv2.minEnclosingCircle(beacon)
        center = (int(x),int(y))
        radius = int(radius)

        if (y < 200) :
            continue

        #also draw a rectangle, we will use the width and height to find aspect ratio (how much like an elipse)
        r1,r2,w,h = cv2.boundingRect(beacon)
        aspect_ratio = float(w)/h
        if (aspect_ratio > 1):
            aspect_ratio = 1
        corrected = radius + (1 - aspect_ratio)*10

        pix_from_center = 320 - int(x)
        if pix_from_center < -20 :
            direction = -1
        elif pix_from_center > 20:
            direction = 1
        else :
            direction = 0

        #print abs(pix_from_center)
        #print float(pix_from_center)/320
        angle = float(abs(pix_from_center))/320*25
        print("angle:",angle)
        

        #Use the radius from drawn circle as a reference
        #Have to apply a corrrective factor of (1 - aspect_ratio)*10 in order to account for possible elipse
        #Use the equasion dist = 8839.2*(corrected)^-1.064 to estimate the distance
             
        #print "Green Radius: %.01f, Ratio: %0.01f Correction: %0.01f" % (radius, aspect_ratio, corrected)
        
        #create distance for sending to arduino
        desiredDistance = int(float(10021)*float(corrected)**-1.089)
        
        #print "Green Distance: %0.01f cm" % (10021*(corrected)**-1.089)
        #print "Angle: %0.01f degrees, %0.01f direction" % (angle, direction)
        image = cv2.circle(image, center, radius, (70, 255, 1), 3)
        green_final = cv2.circle(green_final, center, radius, (70, 255, 1), 3)
        
        #create desiredAngle and send to arduino
        desiredAngle = int(angle)        

        
    arduinoReady = ReceiveData()

    if(arduinoReady == False):
        print("Arduino Ready for data",arduinoReady)
    elif(arduinoReady == True):
        print("Arduino not ready for data",arduinoReady)

    if(direction != 0 and arduinoReady == False):
        print("Direction = ",direction," desiredAngle: %0.01f" % desiredAngle)
        sendData(0,desiredDistance,direction)
            
    if(direction == 0 and arduinoReady == False):
        print("Direction = 0, %0.01f" % desiredDistance)
        sendData(0,desiredDistance,direction)

    if(beacon_flag == False and arduinoReady == False):
        print("Beacon not detected rotating by ",desiredAngle)
        sendData(desiredAngle, 0, direction)
        sleep(1)
        

    
    #saves the image
    cv2.imwrite("green.jpg", green_final)
    cv2.imwrite("Robo-View.jpg", image)

    cv2.namedWindow("Robo-View")
    cv2.namedWindow("Green Filter")

    #shows the image, and wait key takes the keyboard input
    cv2.imshow("Robo-View", image)
    cv2.imshow("Green Filter", green_final)
    
##    if(beacon_flag):
##        LCDSend1("Beacon Detected")
##    else:
##        lcd.clear()
    
    k = cv2.waitKey(1)
    #27 == ESC key
    if k == 27:
        break

#closes all display windows
cv2.destroyAllWindows()

