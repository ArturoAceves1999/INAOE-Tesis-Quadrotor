import halcon as ha
import os
import pyrealsense2 as rs
import numpy as np
import cv2
import time


def variablesStart():
    # Camera values
    xresolution = 640
    yresolution = 480
    framescamera = 30

    # Scale of image values
    MinRange = 0
    MaxRange = 7000
    Multi = 255.0 / (MaxRange - MinRange)

    return xresolution, yresolution, framescamera, Multi


def halconInitialize():
    print("Initializing Halcon programs, please wait...")
    externalcall = 0
    try:
        hdev_engine = ha.HDevEngine()
        hdev_engine.set_attribute("debug_port", 57786)
        hdev_engine.set_attribute("debug_password", "1234")
        hdev_engine.start_debug_server()
        hdev_engine.set_procedure_path(os.getcwd())
        external = ha.HDevProcedure.load_external('processImageHalcon')
        externalcall = ha.HDevProcedureCall(external)
        print("Halcon programs initialized successfully!")
    except:
        externalcall = 0
        print("Halcon programs couldn't be initialized, please check.")
    finally:
        return externalcall


def cameraInitialize(xresolution, yresolution, framescamera):
    print("Initializing camera, please wait...")
    pipeline = 0
    try:
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, xresolution, yresolution, rs.format.z16, framescamera)
        config.enable_stream(rs.stream.color, xresolution, yresolution, rs.format.bgr8, framescamera)
        # Start streaming
        pipeline.start(config)
        print("Camera initialized successfully!")
    except:
        pipeline = 0
        print("Camera couldn't be initialized, please check connections or camera integrity.")
    finally:
        return pipeline


def cameraData(pipeline):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    hole_filling = rs.hole_filling_filter(2)
    filled_depth = hole_filling.process(depth_frame)

    """
    if not depth_frame or not color_frame:
        continue
    """

    # Convert images to numpy arrays
    depth_image = np.asanyarray(filled_depth.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    return depth_image, color_image


def halconProcessing(depthimage, externalcall, multi, oldXYfarthest, oldimagehalcon):
    # now = datetime.datetime.now()
    depthimageHalcon = ha.himage_from_numpy_array(depthimage)
    # print(type(depth_imageHalcon))

    externalcall.set_input_iconic_param_by_name('Z', depthimageHalcon)
    externalcall.set_input_control_param_by_name('Multi', multi)
    externalcall.set_input_control_param_by_name('OldXYFarthestPoint', oldXYfarthest)
    externalcall.set_input_control_param_by_name('OldImage', oldimagehalcon)
    externalcall.execute()
    imagereturnhalcon = externalcall.get_output_iconic_param_by_name('ImageHalcon')
    deptval = externalcall.get_output_control_param_by_name('DeptVal')
    XYfarthest = externalcall.get_output_control_param_by_name('XYFarthestPoint')
    test = externalcall.get_output_control_param_by_name('test')
    print("We have: ", test)
    imagereturnhalcon2 = ha.himage_as_numpy_array(imagereturnhalcon)
    # print(imageReturnHalcon2.shape)
    # print(deptVal)

    return imagereturnhalcon2, XYfarthest, deptval


def showImages(depth_image, imageReturnHalcon2, color_image, deptVal, XYfarthest, midx, midy):
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.rotate(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET),
                                cv2.ROTATE_180)
    color_image = cv2.rotate(color_image, cv2.ROTATE_180)
    depthHALCON_colormap = cv2.applyColorMap(imageReturnHalcon2, cv2.COLORMAP_JET)

    cv2.circle(depthHALCON_colormap, (midx, midy), 4, (0, 0, 255), -1)
    cv2.putText(depthHALCON_colormap, "{} mm".format(deptVal), (midx, midy - 10), 0, 1, (0, 0, 255), 2)
    # cv2.circle(depth_colormap, (midx, midy), 4, (0, 0, 255), -1)
    # cv2.putText(depth_colormap, "{} mm".format(depthpoint), (midx, midy - 10), 0, 1, (0, 0, 255), 2)
    cv2.circle(color_image, (XYfarthest[0], XYfarthest[1]), 4, (0, 0, 255), -1)
    cv2.putText(color_image, "HERE", (XYfarthest[0], XYfarthest[1] - 10), 0, 1, (0, 0, 255), 2)
    # time.sleep(0.05)

    # depth_colormap_dim = depth_colormap.shape
    # color_colormap_dim = color_image.shape

    images = np.hstack((depthHALCON_colormap, color_image))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)


def main():
    [xresolution, yresolution, framescamera, Multi] = variablesStart()
    externalcall = halconInitialize()
    midx = int(xresolution/2)
    midy = int(yresolution/2)
    oldXYfarthest = [0, 0]
    oldimagehalcon =ha.himage_from_numpy_array(np.zeros((xresolution,yresolution)))
    if externalcall == 0:
        exit(0)
    pipeline = cameraInitialize(xresolution, yresolution, framescamera)
    if pipeline == 0:
        exit(0)
    try:
        while True:

            start = time.time()
            [depth_image, color_image] = cameraData(pipeline)
            # depthpoint = int(depth_image[midy, midx])
            [imageReturnHalcon2, XYfarthest, deptVal] = halconProcessing(depth_image, externalcall, Multi, oldXYfarthest, oldimagehalcon)
            print("Farthest point area: ", XYfarthest)
            oldXYfarthest = XYfarthest
            oldimagehalcon = imageReturnHalcon2
            end = time.time()
            print("FPS: ", round(1 / (end - start)))
            showImages(depth_image, imageReturnHalcon2, color_image, deptVal, XYfarthest, midx, midy)
    finally:
        pipeline.stop()


# MAIN CALL
main()
