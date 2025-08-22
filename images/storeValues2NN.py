## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import keyboard
import time

import copy

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
import tqdm






# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)


#640-480-0.5m.txt
#640-480-1m.txt
#640-480-3m.txt
#640-480-5m.txt
#640-480-Maximo.txt
#848-480-0.5m.txt
#848-480-1m.txt
#848-480-3m.txt
#848-480-5m.txt
#848-480-Maximo.txt

#ANTES DE EJECUTAR, RECORDAR DE CAMBIAR LA RESOLUCIÓN

#5.89 max
processGoing = True

model = torch.load("modelo_completo4.pth",weights_only=False)
model.eval()
texto = "640-480-7mNN4FIX"
File_object = open(texto+".txt", "w")
midx = 320
#midx = 424
midy = 240

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
start = time.time()

try:
    while processGoing:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(depth_frame)

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filled_depth.get_data())



        X_new_tensor = torch.tensor(depth_image, dtype=torch.float32).reshape(-1, 1)
        with torch.no_grad():
            y_pred_test = model(X_new_tensor).numpy()
        depth_image = y_pred_test.reshape(depth_image.shape)


        color_image = np.asanyarray(color_frame.get_data())
        depthpoint = depth_image[midy, midx]
        end = time.time()
        File_object.write(f"{depthpoint} {end -start}\n")
        print(end -start)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # If depth and color resolutions are different, resize color image to match depth image for display


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', depth_colormap)
        cv2.waitKey(1)
        if keyboard.is_pressed("q"):
            processGoing = False
            File_object.close()
            cv2.imwrite(texto+'.png', depth_colormap)


finally:

    # Stop streaming
    pipeline.stop()