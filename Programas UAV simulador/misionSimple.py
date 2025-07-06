from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import random
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
#from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import argparse
import datetime


class ImprimirSubscriber(Node):
    def __init__(self):
        super().__init__('impresor_suscriptor')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mensaje_teclado',
            self.listener_callback,
            1)
        self.last_msg = [0.0, 0.0, 0.0, 0.0]

    def listener_callback(self, msg):
        self.last_msg = msg.data


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)

def get_location_metres_simple(archi1, location, takeofflocation, isevading):
    earth_radius = 6378137.0
    
    dlat = math.radians(location.lat - takeofflocation.lat)
    dlon = math.radians(location.lon - takeofflocation.lon)
    x = dlon * earth_radius * math.cos(math.radians(takeofflocation.lat))
    y = dlat * earth_radius
    
    archi1.write(str(x) + " " + str(y) + " " + str(location.alt) + " " + str(isevading)  + "\n") 

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def distance_to_current_waypoint(vehicle):
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint - 1]
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    return get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)


def adds_square_mission(vehicle, aLocation, aSize):
    cmds = vehicle.commands
    print(" Clear any existing commands")
    cmds.clear()

    print(" Define/add new commands.")
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    point1 = get_location_metres(aLocation, aSize, 0)
    point2 = get_location_metres(aLocation, aSize+aSize, aSize)
    #point3 = get_location_metres(aLocation, -aSize, aSize)
    #point4 = get_location_metres(aLocation, -aSize, -aSize)
    point4 = get_location_metres(aLocation, 0, 0)
    for idx, point in enumerate([point1, point2, point4]):
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         point.lat, point.lon, 2))
    # Dummy WP 5
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                     point4.lat, point4.lon, 2))

    print(" Upload new commands to vehicle")
    cmds.upload()


def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.8:
            print("Reached target altitude")
            break
        time.sleep(1)
        
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        2503,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    
    vehicle.send_mavlink(msg)
    vehicle.flush()
    vehicle.airspeed = 0.5
    
    time.sleep(5)
    

def get_distance_local_ned(current, target):
    dx = target[0] - current.north
    dy = target[1] - current.east
    dz = target[2] - current.down
    return math.sqrt(dx**2 + dy**2 + dz**2)

def goto_evation(vehicle, directions, archi1, originallocation):
    print("Entering EVATION mode")
    vehicle.mode = VehicleMode("GUIDED")
    
    forward = directions[2]
    right = directions[0]
    down = -directions[1]
    
    yaw_deg = vehicle.heading
    yaw_rad = math.radians(yaw_deg)
    north = math.cos(yaw_rad) * forward - math.sin(yaw_rad) * right
    east  = math.sin(yaw_rad) * forward + math.cos(yaw_rad) * right
    loc = vehicle.location.local_frame
    target = (loc.north + north, loc.east + east, loc.down + down)
    
    print(f"Evasión hacia (N, E, D): {target}")
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        1528,
        target[0], target[1], target[2], 
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    
    for _ in range(20):  # máx 5 segundos
        vehicle.send_mavlink(msg)
        vehicle.flush()
        vehicle.airspeed = 0.5
        loc = vehicle.location.local_frame
        dist = math.sqrt((target[0] - loc.north)**2 + (target[1] - loc.east)**2 + (target[2] - loc.down)**2)
        get_location_metres_simple(archi1, vehicle.location.global_relative_frame, originallocation,1)
        print(f"Distancia restante: {dist:.2f} m")

        if dist < 0.1:
            print("Evasión completada.")
            break
            
        time.sleep(0.1)
    
    vehicle.mode = VehicleMode("AUTO")
    vehicle.airspeed = 0.5


def main():
    parser = argparse.ArgumentParser(description='Misión simple con DroneKit y ROS 2.')
    parser.add_argument('--connect', help="Cadena de conexión al vehículo.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None
    date = datetime.datetime.now()
    archi1=open("LOG " + date.strftime('%c'),"w") 

    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    print('Creando misión')
    adds_square_mission(vehicle, vehicle.location.global_frame, 5)
    arm_and_takeoff(vehicle, 2)
    originallocation = vehicle.location.global_relative_frame 
    get_location_metres_simple(archi1, originallocation,originallocation, 0)
    
    print("Iniciando misión")
    vehicle.commands.next = 0
    vehicle.mode = VehicleMode("AUTO")
    vehicle.airspeed = 0.5

    # Inicializar ROS una vez
    rclpy.init()
    nodo = ImprimirSubscriber()

    try:
        while True:
            get_location_metres_simple(archi1, vehicle.location.global_relative_frame, originallocation, 0)
            rclpy.spin_once(nodo, timeout_sec=0.1)

            if nodo.last_msg:
                data = nodo.last_msg
                nodo.last_msg = None
                evationmode = int(data[0])
                directions = data[1:]
                print(f'Data: "{directions}"')
                
                if evationmode == 1:
                    print("¡Comando de evasión recibido!")
                    goto_evation(vehicle, directions,archi1, originallocation)
                    nodo.last_msg = None
                elif evationmode == 2:
                    print("¡Comando de evasión EMERGENCIA recibido!")
                    goto_evation(vehicle, [0,0,-1.5], archi1, originallocation)
                    nodo.last_msg = None
                elif evationmode == 0:
                    print("No es necesario evadir")
                    nodo.last_msg = None

            nextwaypoint = vehicle.commands.next
            vehicle.airspeed = 0.5
            print(f'Distance to waypoint ({nextwaypoint}): {distance_to_current_waypoint(vehicle)}')

            if nextwaypoint == 3:
                print("Finalizando misión")
                break

            time.sleep(1)

    finally:
        print('Return to launch')
        vehicle.mode = VehicleMode("RTL")
        print("Closing vehicle")
        vehicle.close()

        if sitl is not None:
            sitl.stop()

        nodo.destroy_node()
        rclpy.shutdown()
        archi1.close() 

