from pymavlink import mavutil
import serial
import time
from math import sqrt
import argparse
import sys
import threading

POSITION_ONLY = 0b110111111000
VELOCITY_ONLY = 0b110111000111
GUIDED = 4

class DroneState:
    def __init__(self):
        self.heartbeat_received = False
        self.mode_guided = True
        self.armed = False
        self.relative_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.battery_low = False
        self.position_lost = False
        self.landed = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.system_fault = 0

drone = DroneState()
parser = argparse.ArgumentParser()
# 2. Define positional parameters (Required by default)
parser.add_argument("--drone", type=str, help="Physical/IP port")
parser.add_argument("--serial", type=str, help="Physical/IP port")
args = parser.parse_args()

def send_command(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    """
    0 (MAV_RESULT_ACCEPTED): The command is valid and was executed.
    1 (MAV_RESULT_TEMPORARILY_REJECTED): The command is valid, but the drone is not in the right state.
    2 (MAV_RESULT_DENIED): The drone understood the command but refused to execute it.
    3 (MAV_RESULT_UNSUPPORTED): The command is invalid or not supported by your vehicle's firmware.
    4 (MAV_RESULT_FAILED): The command was attempted but failed
    """
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        cmd,
        0,
        p1,
        p2, p3, p4, p5, p6, p7,
    )
    timeout = 5
    start_time = time.time()
    ack = None
    while time.time() - start_time < timeout:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack is not None and ack.command == cmd:
            return ack.result
        time.sleep(0.2)
    return None

def set_local_position(bitmask, north, east, down, vel, accel, yaw):
    """
    0 = active 1 = ignore
    Bit 0-2 pos x, y, z
    Bit 3-5 velocity x, y, z
    Bit 6-8 accel x, y, z
    bit 9: 0 for normal 1 for force
    bit 10-11: yaw target, rate
    Bit 12-15: unused
    """
    conn.mav.set_position_target_local_ned_send(
        0, # Boot time (ignored)
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Fixed local frame: +X North, +Y East, +Z Down
        bitmask, 
        north,  # X: Position North (meters)
        east,  # Y: Position East (Negative = 5m West)
        down,   # Z: Position Down (0.0 to stay at current altitude)
        vel[0], vel[1], vel[2], # X, Y, Z velocity (ignored by bitmask)
        accel[0], accel[1], accel[2], # X, Y, Z acceleration (ignored by bitmask)
        yaw[0], yaw[1]     # Yaw target, Yaw rate (ignored by bitmask)
    )
    return True
    
def reached_position(north, east, down, x, y, z, tolerance=1):
    distance = sqrt((north - x) ** 2 + (east - y) ** 2 + (down - z) ** 2)
    return distance <= tolerance

# negative value for down = positive altitude
def goto_local_ned(north, east, down, timeout, tolerance=1):
    set_local_position(POSITION_ONLY, north, east, down, (0, 0, 0), (0, 0, 0), (0, 0))
    start_time = time.time()
    while time.time() - start_time <= timeout:
        if reached_position(north, east, down, drone.x, drone.y, drone.z, tolerance):
            return True
    return False

def stop():
    msg = conn.mav.set_position_target_local_ned_encode(
            time_boot_ms=0,
            target_system=conn.target_system,
            target_component=conn.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,    # MAV_FRAME_BODY_NED
            type_mask=VELOCITY_ONLY,           
            x=0.0, y=0.0, z=0.0,      
            vx=0, vy=0, vz=0, 
            afx=0.0, afy=0.0, afz=0.0,
            yaw=0.0, yaw_rate=0.0     
        )
    conn.mav.send(msg)
    return True

def relative_motion(vx, vy, vz, duration):
    start = time.time()
    while time.time() - start < duration:
        msg = conn.mav.set_position_target_local_ned_encode(
            time_boot_ms=0,
            target_system=conn.target_system,
            target_component=conn.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,    # MAV_FRAME_BODY_NED
            type_mask=VELOCITY_ONLY,           
            x=0.0, y=0.0, z=0.0,      
            vx=vx, vy=vy, vz=vz, 
            afx=0.0, afy=0.0, afz=0.0,
            yaw=0.0, yaw_rate=0.0     
        )
        conn.mav.send(msg)
        time.sleep(0.2)
    stop()
    return True

def absolute_motion(v_north, v_east, v_down, duration):
    start = time.time()
    while time.time() - start < duration:
        msg = conn.mav.set_position_target_local_ned_encode(
            time_boot_ms=0,
            target_system=conn.target_system,
            target_component=conn.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # MAV_FRAME_LOCAL_NED
            type_mask=VELOCITY_ONLY,           
            x=0.0, y=0.0, z=0.0,      
            vx=v_north, vy=v_east, vz=v_down, 
            afx=0.0, afy=0.0, afz=0.0,
            yaw=0.0, yaw_rate=0.0     
        )
        conn.mav.send(msg)
        time.sleep(0.2)
    stop()
    return True

def error_handle(foo, *args):
    if foo(*args) == False:
        print(f"Error: {foo.__name__} failed")
        if ser and ser.is_open:
            ser.write(f"Error: {foo.__name__} failed")
        land(30)
        sys.exit()

def arm(timeout):
    cmd_accepted = send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1) == 0
    if not cmd_accepted:
        return False
    start_time = time.time()
    while True:
        if time.time() - start_time >= timeout:
            return False
        if drone.armed:
            return True
        time.sleep(0.2)

def disarm(timeout):
    cmd_accepted = send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0) == 0
    if not cmd_accepted:
        return False
    start_time = time.time()
    while time.time() - start_time <= timeout:
        if not drone.armed:
            return True
        time.sleep(0.2)
    return False

def takeoff(altitude, timeout, tolerance=1):
    cmd_accepted = send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=altitude) == 0
    if not cmd_accepted:
        return False
    start_time = time.time()
    while time.time() - start_time <= timeout:
        if abs(drone.relative_alt - altitude) <= tolerance:
            return True
        time.sleep(0.2)
    return False
    
def land(timeout):
    cmd_accepted = send_command(mavutil.mavlink.MAV_CMD_NAV_LAND) == 0
    if not cmd_accepted:
        return False
    start_time = time.time()
    while time.time() - start_time <= timeout:
        if drone.landed:
            return True
        time.sleep(0.2)
    return False

def wait_for_guided(timeout):
    start_time = time.time()
    while time.time() - start_time <= timeout:
        conn.wait_heartbeat()
        if drone.mode_guided == True:
            return True
        time.sleep(0.2)
    return False

drone_conn = args.drone or "udp:127.0.0.1:14550"
conn = mavutil.mavlink_connection(drone_conn)

ser = None
try:
    port = args.serial
    if port is not None:
        ser = serial.Serial(port=port, baudrate=115200, timeout=3)
except:
    print("No serial connection")
time.sleep(2)

def main():
    conn.wait_heartbeat()
    conn.set_mode("GUIDED")
    error_handle(wait_for_guided, 10)
    error_handle(arm, 10)
    error_handle(takeoff, 10, 60, 1)
    error_handle(goto_local_ned, 5, 4, -10, 20)
    error_handle(relative_motion, 0.5, 0, 0, 2)
    error_handle(absolute_motion, 0.5, 0.2, 0, 4)
    error_handle(land, 45)

def supervise():
    while 1:
        msg = conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
        if msg is None:
            drone.position_lost = None
        else:
            drone.position_lost = msg.h_acc >= 3000
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2.0)
        if msg is None:
            drone.relative_alt = None
            drone.vx = None
            drone.vy = None
            drone.vz = None
        else:
            drone.relative_alt = msg.relative_alt / 1000.0
            drone.vx = msg.vx / 100.0
            drone.vy = msg.vy / 100.0
            drone.vz = msg.vz / 100.0
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2.0)
        if msg is None:
            drone.x = None
            drone.y = None
            drone.z = None
        else:
            drone.x = msg.x
            drone.y = msg.y
            drone.z = msg.z
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg is None:
            drone.heartbeat_received = None
            drone.mode_guided = None
            drone.armed = None
        else:
            drone.heartbeat_received = True
            drone.mode_guided = msg.custom_mode == GUIDED
            drone.armed = msg.base_mode == 128
        msg = conn.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=2.0)
        if msg is None:
            drone.landed = None
        else:
            drone.landed = msg.landed_state == 1 
        msg = conn.recv_match(type="SYS_STATUS", blocking=True, timeout=2.0)
        if msg is None:
            drone.battery_low = None
        else:
            drone.battery_low = msg.battery_remaining < 30

        if not drone.heartbeat_received:
            drone.system_fault |= (0b1 << 0)
        else:
            drone.system_fault &= ~(0b1 << 0)
        if not drone.mode_guided:
            drone.system_fault |= (0b1 << 1)
        else:
            drone.system_fault &= ~(0b1 << 1)
        if not drone.armed:
            drone.system_fault |= (0b1 << 2)
        else:
            drone.system_fault &= ~(0b1 << 2)
        if drone.battery_low:
            drone.system_fault |= (0b1 << 7)
        else:
            drone.system_fault &= ~(0b1 << 7)
        if drone.position_lost:
            drone.system_fault |= (0b1 << 8)
        else:
            drone.system_fault &= ~(0b1 << 8)
        time.sleep(0.2)

threading.Thread(target=main).start()
threading.Thread(target=supervise).start()
