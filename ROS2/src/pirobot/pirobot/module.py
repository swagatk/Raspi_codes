#This is a support Module mainly for the speech from text and also to aid the
#System into knowing when a finger is up or when a finger is down based on the
#joints

import cv2
import mediapipe
#import os

drawingModule = mediapipe.solutions.drawing_utils
handsModule = mediapipe.solutions.hands

mod=handsModule.Hands()


h=480
w=640


def findpostion(frame):
    list=[]
    results = mod.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks != None:
        for handLandmarks in results.multi_hand_landmarks:
            drawingModule.draw_landmarks(frame, handLandmarks, handsModule.HAND_CONNECTIONS)
            list=[]
            for id, pt in enumerate (handLandmarks.landmark):
                x = int(pt.x * w)
                y = int(pt.y * h)
                list.append([id,x,y])

    return list            





def findnameoflandmark(frame):
    list=[]
    results = mod.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks != None:
#        for handLandmarks in results.multi_hand_landmarks:


        for point in handsModule.HandLandmark:
            list.append(str(point).replace ("< ","").replace("HandLandmark.", "").replace("_"," ").replace("[]",""))
    return list




