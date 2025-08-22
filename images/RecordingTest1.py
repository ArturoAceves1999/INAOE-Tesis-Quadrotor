import pyrealsense2 as rs
import numpy as np
import cv2
#import datetime
import keyboard








from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression


'''
poly = PolynomialFeatures(degree=1, include_bias=True)
poly_reg_model = LinearRegression()

input_file = "salidaFORMC.txt"
x_train, x_test, x_valid, y = np.loadtxt(input_file, unpack=True)

poly_features = poly.fit_transform(x_train.reshape(-1, 1))
poly_reg_model.fit(poly_features, y)
y_predicted = poly_reg_model.predict(poly_features)
'''

import copy
import torch
import torch.nn as nn
import torch.optim as optim
import tqdm

model = torch.load("modelo_completo4.pth",weights_only=False)
model.eval()





pipeline = rs.pipeline()
config = rs.config()

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

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


processGoing = True
# Start streaming
pipeline.start(config)

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
        color_image = np.asanyarray(color_frame.get_data())

        print(type(color_frame.get_data()))
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
       # now = datetime.datetime.now()
        midx = 320
        midy = 240
        depthpoint = depth_image[midy,midx]
        print(depthpoint)

        #depth_image = depth_image * np.exp(depth_image * -0.00001)
        #depth_image = (-0.000010455 * (depth_image ** 2)) + (1.0271 * depth_image) - 22.7636

        '''
        shape = depth_image.shape
        poly_features = poly.transform(depth_image.reshape(-1, 1))
        y_predicted = poly_reg_model.predict(poly_features)
        depth_image = y_predicted.reshape(depth_image.shape)
        '''

        batch_size = 10000  # número de elementos por batch
        depth_flat = depth_image.reshape(-1, 1)  # aplana
        y_pred_flat = np.zeros_like(depth_flat)  # para guardar resultados
        with torch.no_grad():
            for start in range(0, depth_flat.shape[0], batch_size):
                end = start + batch_size
                batch = torch.tensor(depth_flat[start:end], dtype=torch.float32)
                y_pred_flat[start:end] = model(batch).numpy()

        # reconstruir la forma original
        depth_image = y_pred_flat.reshape(depth_image.shape)




        '''
        X_new_tensor = torch.tensor(depth_image, dtype=torch.float32).reshape(-1, 1)
        with torch.no_grad():
            y_pred_test = model(X_new_tensor).numpy()
        depth_image = y_pred_test.reshape(depth_image.shape)
        '''

        depthpoint = depth_image[midy, midx]
        #print(depthpoint)
        cv2.circle(color_image,(midx,midy),4,(0,0,255),-1)
        cv2.putText(color_image,"{} mm".format(depthpoint),(midx,midy-10),0,1,(0,0,255),2)
        cv2.circle(depth_colormap,(midx,midy),4,(0,0,255),-1)
        cv2.putText(depth_colormap,"{} mm".format(depthpoint),(midx,midy-10),0,1,(0,0,255),2)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        #print("Shape: ", depth_colormap.shape)

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

    if keyboard.is_pressed('q'):
        print(processGoing)
        processGoing = False

finally:

    # Stop streaming
   pipeline.stop()