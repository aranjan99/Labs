#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np
import functools
import math
import time
import cozmo
import imgclassification
from cozmo.util import degrees, distance_mm, radians, speed_mmps, Vector2
from cozmo.util import degrees, Pose
try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    label_old = "none"
        
    duration_s = 5 
    img_clf = imgclassification.ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    train_data = img_clf.extract_image_features(train_raw)
    img_clf.train_classifier(train_data, train_labels)
    print("model trained")
    count = 0

    try:
        while True:
        
           #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            
            image = np.array(event.image)
            cv2.imwrite("./robot/drone_10T094634322772.bmp", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        
            (test_raw, test_labels) = img_clf.load_data_from_folder('./robot/') 
            test_data = img_clf.extract_image_features(test_raw)
            label_new = img_clf.predict_labels(test_data)
            print("here")
            print(label_new[0])
            if(label_old == label_new[0]):
               count = count + 1
            else:
               count = 0
            label_old = label_new[0]
            if(label_old == "plane" and count == 2):
                robot.drive_straight(distance_mm(150), speed_mmps(50)).wait_for_completed()
                time.sleep(duration_s)
                robot.say_text(label_old, voice_pitch=-1.0, duration_scalar=0.3).wait_for_completed()
                time.sleep(duration_s)
            elif(label_old == "truck" and count == 2):    
                robot.drive_straight(distance_mm(-150), speed_mmps(50)).wait_for_completed()                
                time.sleep(duration_s)
                robot.say_text(label_old, voice_pitch=-1.0, duration_scalar=0.3).wait_for_completed()
                time.sleep(duration_s)
            elif(label_old == "order" and count == 2):
                robot.move_head(-0.15)
                time.sleep(duration_s)                
                robot.say_text(label_old, voice_pitch=-1.0, duration_scalar=0.3).wait_for_completed()
                time.sleep(duration_s)
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

