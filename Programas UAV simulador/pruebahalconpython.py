import halcon as ha
import os
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
#from types import SimpleNamespace


class messagePublisher(Node):
    def __init__(self):
        super().__init__('teclado_publicador')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mensaje_teclado', 1)

    def send_message(self, value):
    
        msg = Float32MultiArray()         
        msg.data = value
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publicado: "{texto}"')


def initvariables():
    class initvar:
        xresolution = 640
        yresolution = 480
        FOVXcamera = 87
        framescamera = 30
        numdivisions = 20
        diffXYpoint = 20

        # Scale of image values
        minrange = 0
        maxrange = 7000
        multi = 255.0 / (maxrange - minrange)
        tresholdmax = 2500
        tresholdmin = 550
        tresholdstop = 500
        areachange = 4000 #Range between 0-307,200 if 640X480. If bigger, the image will need an even bigger change between frames to be modified
        percentevasion = 0.8
        areaevasion = int(percentevasion*(xresolution*yresolution)) #Range between 0-307,200 if 640X480
        # Drone dimension
        frameradius = 330 #On mm

        frameratio = (xresolution*frameradius)/(2*math.tan(math.radians(FOVXcamera/2)))
        sensibility = 1.1
        secdistanceincrease = 0.9 #Used for increasing distance values on X and Y for evasion. If bigger, the incresing on short distances is going to be bigger, but smaller on longer distances


    return initvar


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
    except Exception as e:
        externalcall = 0
        print(f"Halcon programs couldn't be initialized, please check: {e}")
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
    except Exception as e:
        pipeline = 0
        print(f"Camera couldn't be initialized, please check: {e}")
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
    depth_image = depth_image * np.exp(depth_image * -0.00001)
    depth_image = depth_image.astype(np.uint16)
    color_image = np.asanyarray(color_frame.get_data())
    return depth_image, color_image


def halconProcessing(depthimage, externalcall, initialVariables, oldXYfarthest, oldimagehalcon):
    # now = datetime.datetime.now()
    depthimageHalcon = ha.himage_from_numpy_array(depthimage)
    oldimagehalcon = ha.himage_from_numpy_array(oldimagehalcon)
    # print(type(depth_imageHalcon))

    externalcall.set_input_iconic_param_by_name('Z', depthimageHalcon)
    externalcall.set_input_iconic_param_by_name('OldFrame', oldimagehalcon)
    externalcall.set_input_control_param_by_name('Multi', initialVariables.multi)
    externalcall.set_input_control_param_by_name('TresholdDistance', initialVariables.tresholdmax)
    externalcall.set_input_control_param_by_name('AreaChange', initialVariables.areachange)
    externalcall.set_input_control_param_by_name('AreaEvasion', initialVariables.areaevasion)
    externalcall.set_input_control_param_by_name('OldXYFarthestPoint', oldXYfarthest)
    externalcall.set_input_control_param_by_name('UAVFrameRatio', initialVariables.frameratio)
    externalcall.set_input_control_param_by_name('TresholdDistanceMin', initialVariables.tresholdmin)
    externalcall.set_input_control_param_by_name('NumIntervals', initialVariables.numdivisions)
    externalcall.set_input_control_param_by_name('DilationSensibility', initialVariables.sensibility)
    externalcall.set_input_control_param_by_name('TresholdDistanceStop', initialVariables.tresholdstop)
    externalcall.set_input_control_param_by_name('DifferenceXYPoint', initialVariables.diffXYpoint)
    externalcall.set_input_control_param_by_name('SecurityDistanceIncrease', initialVariables.secdistanceincrease)
    
    externalcall.execute()
    imagereturnhalcon = externalcall.get_output_iconic_param_by_name('ImageHalcon')
    deptval = externalcall.get_output_control_param_by_name('DeptVal')
    XYfarthest = externalcall.get_output_control_param_by_name('XYFarthestPoint')
    test = externalcall.get_output_control_param_by_name('test')
    evasionmode = externalcall.get_output_control_param_by_name('EvasionMode')
    newdistance = externalcall.get_output_control_param_by_name('NewDistance')
    imagereturnhalcon2 = ha.himage_as_numpy_array(imagereturnhalcon)
    # print(imageReturnHalcon2.shape)
    # print(deptVal)

    return imagereturnhalcon2, XYfarthest, deptval, evasionmode, newdistance


def showImages(depth_image, imageReturnHalcon2, color_image, deptVal, XYfarthest, evasionmode, midx, midy):
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
    cv2.putText(color_image, "EVASION MODE: {}".format(evasionmode[0]), (20,40), 0, 1, (0, 0, 255), 2)
    # time.sleep(0.05)

    # depth_colormap_dim = depth_colormap.shape
    # color_colormap_dim = color_image.shape

    images = np.hstack((depthHALCON_colormap, color_image))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)


def main(args=None):
    #[xresolution, yresolution, framescamera, multi, treshold] = variablesStart()
    externalcall = halconInitialize()
    initialVariables = initvariables()
    midx = int(initialVariables.xresolution/2)
    midy = int(initialVariables.yresolution/2)
    oldXYfarthest = [0, 0]
    flagevasion = False
    latestevation = 0
    latespoint = [0, 0]
    latesdirection = [0.0, 0.0, 0.0]
    stabilitytimerstart = 0
    stabilitytimerend = 0
    x=0.0
    y=0.0
    z=0.0


    oldimagehalcon =np.zeros((initialVariables.xresolution,initialVariables.yresolution), dtype=np.int8)
    if externalcall == 0:
        exit(0)
    pipeline = cameraInitialize(initialVariables.xresolution, initialVariables.yresolution, initialVariables.framescamera)
    if pipeline == 0:
        exit(0)
        
    rclpy.init(args=args)
    nodo = messagePublisher()
    
    
    try:
        while True:

            start = time.time()
            [depth_image, color_image] = cameraData(pipeline)
            # depthpoint = int(depth_image[midy, midx])
            [imageReturnHalcon2, XYfarthest, deptVal, evasionmode, newdistance] = halconProcessing(depth_image, externalcall, initialVariables, oldXYfarthest, oldimagehalcon)
            

            if (evasionmode[0] == 1) and (flagevasion == False):
                stabilitytimerstart = time.time()
                latestevation = evasionmode
                latespoint = XYfarthest
                latesdirection = newdistance
                flagevasion = True

            if flagevasion:
                evasionmode = latestevation
                XYfarthest = latespoint
                newdistance = latesdirection

            #print("Farthest point area: ", XYfarthest)
            print(newdistance)
            oldXYfarthest = XYfarthest
            oldimagehalcon = imageReturnHalcon2
            end = time.time()
            stabilitytimerend = time.time()
            if (stabilitytimerend - stabilitytimerstart) > 1.0:
                flagevasion = False
            #print("FPS: ", round(1 / (end - start)))
            print(type(newdistance[0]))
            showImages(depth_image, imageReturnHalcon2, color_image, deptVal, XYfarthest, evasionmode, midx, midy)
            
            
            value = [float(evasionmode[0]), newdistance[0], newdistance[1], newdistance[2]]
            
            nodo.send_message(value)
    except KeyboardInterrupt:
        print("Closing program")
    finally:
        pipeline.stop()
        nodo.destroy_node()
        rclpy.shutdown()


# MAIN CALL
main()
