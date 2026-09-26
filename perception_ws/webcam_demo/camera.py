import numpy as np
import cv2 as cv
import mediapipe as mp
from ultralytics import YOLO
from PIL import Image

cap = cv.VideoCapture(0, cv.CAP_V4L2) 
model = YOLO("cardiff-autonomous-racing/perception_ws/models/best.pt") # im gonna train a faster model for cpu

if not cap.isOpened():

    print("Cannot open camera")
    exit()

count = 1
results = None

while True:
    ret, frame = cap.read()
    
    if not ret or frame is None:
        print("Can't read frame")
        break  # exit the loop if stream drops

    if count % 3 == 0 or results == None:
        results = model.predict(frame, conf = 0.5, imgsz = 320) # can make this smaller if you want better performance

    for i, r in enumerate(results):
        im_bgr = r.plot()
        cv.imshow("cone", im_bgr)
    # wait for 1 millisecond per frame. If q is pressed, exit.
    count+=1
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
