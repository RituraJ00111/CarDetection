import cv2
import numpy as np 
import math 
from object_detection_edit import ObjectDetection

#defining function for calculating the dist.
def calc(x1,x2,y1,y2):
    return math.hypot(x2-x1,y2-y1)

#Load object detection
od = ObjectDetection()


cam = cv2.VideoCapture("los_angeles.mp4")
alive = True
count = 0
center_point_prev_frame = []
tracking_id = 0
tracking_object = {}

while alive:
    ret, frame = cam.read()
    #My phone dim is (720, 1280, 3)
    #height,width,_ = framer.shape
    #frame = framer[give required values]

    count +=1

    #Tracking
    center_point_cur_frame = []
    # Detecting the frame and recieving the box infos..
    (classes,scores,boxes) = od.detect(frame)
    for box in boxes:
        (x,y,w,h) = box
        cx = int((x+x+w)/2)
        cy = int((y+y+h)/2)
        center_point_cur_frame.append((cx,cy))
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        #cv2.circle(frame,(cx,cy),4,(0,0,255),-1)
    
    #Calculating the distance
    if count <= 2:
        for pt in center_point_cur_frame:
            for pt2 in center_point_prev_frame:
                distance = calc(pt2[0],pt[0],pt2[1],pt[1])

                if distance<20:
                    tracking_object[tracking_id] = pt
                    tracking_id += 1
    else:
        center_point_cur_frame_copy = center_point_cur_frame.copy()
        tracking_object_copy = tracking_object.copy()

        for obj_id, pt2 in tracking_object_copy.items():
            object_exists = False

            for pt in center_point_cur_frame_copy:
                distance = calc(pt[0],pt2[0],pt[1],pt2[1])

                if distance <20 :
                    tracking_object[obj_id] = pt
                    object_exists = True
                    if pt in center_point_cur_frame:
                        center_point_cur_frame.remove(pt)
                    continue
            if not object_exists:
                tracking_object.pop(obj_id)
            
        
        #Adding new ids
        for pt in center_point_cur_frame:
            for pt2 in center_point_prev_frame_t:
                distance = calc(pt[0],pt2[0],pt[1],pt2[1])
                if distance <20:
                    tracking_object[tracking_id] = pt
                    tracking_id += 1

    for object_id, pt in tracking_object.items():
        cv2.circle(frame, pt, 5, (0, 0, 255), -1)
        cv2.putText(frame, str(object_id), (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 2)

    center_point_prev_frame_t = center_point_cur_frame.copy()
    center_point_prev_frame = center_point_cur_frame.copy()

    
    
    cv2.imshow("window1",frame)
    if cv2.waitKey(1) == 27:
        alive = False

cam.release()
cv2.destroyAllWindows()
