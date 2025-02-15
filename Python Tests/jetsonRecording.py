## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time

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
File_object = open("640-480-FRAMETEST.txt", "w")
midx = 320
#midx = 424
midy = 240
framenumber = 1

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while framenumber < 32:
        if framenumber == 1:
            frames = pipeline.wait_for_frames()
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            hole_filling = rs.hole_filling_filter()
            filled_depth = hole_filling.process(depth_frame)

            # Convert images to numpy arrays
            depth_image = np.asanyarray(filled_depth.get_data())
            depth_image = depth_image * np.exp(depth_image * -0.00001)
            depthpoint = depth_image[midy, midx]
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            framenumber = framenumber + 1
            startglobal = time.time()
        else:
            start = time.time()
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            hole_filling = rs.hole_filling_filter()
            filled_depth = hole_filling.process(depth_frame)

            # Convert images to numpy arrays
            depth_image = np.asanyarray(filled_depth.get_data())
            depth_image = depth_image * np.exp(depth_image * -0.00001)
            depthpoint = depth_image[midy, midx]
            end = time.time()
            File_object.write(f"{end - startglobal} {end - start} {framenumber - 1}\n")
            framenumber = framenumber + 1


finally:

    # Stop streaming
    pipeline.stop()