import halcon as ha
import os
import pyrealsense2 as rs
import numpy as np
import cv2


def variablesStart():
    # Camera values
    xresolution = 640
    yresolution = 480
    framescamera = 30

    # Scale of image values
    MinRange = 0
    MaxRange = 5000
    Multi = 255.0 / (MaxRange - MinRange)

    return xresolution, yresolution, framescamera, Multi

def halconInitialize():
    print("Initializing Halcon programs, please wait...")
    externalCall = 0
    try:
        hdev_engine = ha.HDevEngine()
        hdev_engine.set_attribute("debug_port", 57786)
        hdev_engine.set_attribute("debug_password", "1234")
        hdev_engine.start_debug_server()
        hdev_engine.set_procedure_path(os.getcwd())
        external = ha.HDevProcedure.load_external('processImageHalcon')
        externalCall = ha.HDevProcedureCall(external)
        print("Halcon programs initialized successfully!")
    except:
        externalCall = 0
        print("Halcon programs couldn't be initialized, please check.")
    finally:
        return externalCall

def cameraInitialize(xresolution, yresolution, framescamera):
    print("Initializing camera, please wait...")
    pipeline = 0
    try:
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
        config.enable_stream(rs.stream.color, xresolution, yresolution, rs.format.bgr8, framescamera)
        # Start streaming
        pipeline.start(config)
        print("Camera initialized successfully!")
    except:
        pipeline = 0
        print("Camera couldn't be initialized, please check connections or camera integrity.")
    finally:
        return pipeline

def cameraData(pipeline, externalCall, Multi):
    try:
        while True:

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

            # now = datetime.datetime.now()
            midx = 320
            midy = 240
            depthpoint = int(depth_image[midy, midx])
            depth_imageHalcon = ha.himage_from_numpy_array(depth_image)
            # print(type(depth_imageHalcon))

            externalCall.set_input_iconic_param_by_name('Z', depth_imageHalcon)
            externalCall.set_input_control_param_by_name('Multi', Multi)
            externalCall.execute()
            imageReturnHalcon = externalCall.get_output_iconic_param_by_name('ImageHalcon')
            deptVal = externalCall.get_output_control_param_by_name('DeptVal')
            imageReturnHalcon2 = ha.himage_as_numpy_array(imageReturnHalcon)
            #print(imageReturnHalcon2.shape)
            #print(deptVal)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depthHALCON_colormap = cv2.applyColorMap(imageReturnHalcon2, cv2.COLORMAP_JET)

            cv2.circle(depthHALCON_colormap, (midx, midy), 4, (0, 0, 255), -1)
            cv2.putText(depthHALCON_colormap, "{} mm".format(deptVal), (midx, midy - 10), 0, 1, (0, 0, 255), 2)
            cv2.circle(depth_colormap, (midx, midy), 4, (0, 0, 255), -1)
            cv2.putText(depth_colormap, "{} mm".format(depthpoint), (midx, midy - 10), 0, 1, (0, 0, 255), 2)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)
                images = np.hstack((depthHALCON_colormap, depth_colormap))
            else:
                images = np.hstack((depthHALCON_colormap, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)


    finally:

        # Stop streaming
        pipeline.stop()

def main():
    [xresolution, yresolution, framescamera, Multi] = variablesStart()
    externalcal = halconInitialize()
    if externalcal == 0:
        exit(0)
    pipeline = cameraInitialize(xresolution, yresolution, framescamera)
    if pipeline == 0:
        exit(0)
    cameraData(pipeline, externalcal,Multi)

#MAIN CALL
main()
