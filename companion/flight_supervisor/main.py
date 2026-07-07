from pymavlink import mavutil
import serial
import time
from math import sqrt
import argparse
import sys
import threading
from dataclasses import dataclass 

POSITION_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

VELOCITY_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

NONE = 0
GUIDED = 4
TIMEOUT = 3.0

@dataclass
class Fault:
    STALE_HEARTBEAT = 0x1
    STALE_POSITION_LOST = 0x2
    STALE_GLOBAL_VELOCITY = 0x3
    STALE_LOCAL_POSITION = 0x4
    STALE_LANDED = 0x5
    STALE_BATTERY_LOW = 0x6
    FAULT_POSITION_LOST = 0x7
    FAULT_BATTERY_LOW = 0x8
    FAULT_DISARMED = 0x9
    FAULT_NOT_GUIDED = 0xA


class DroneState:
    def __init__(self):
        self.heartbeat_received = False
        self.heartbeat_timestamp = 0
        self.mode_guided = True
        self.armed = False
        self.relative_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.velocity_timestamp = 0
        self.battery_low = False
        self.battery_timestamp = 0
        self.position_lost = False
        self.position_lost_start = 1
        self.position_lost_timestamp = 0
        self.unknown_position_timer = 0
        self.landed = False
        self.landed_timestamp = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.position_timestamp = 0
        self.system_fault = 0
        self.commands = dict()


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
    drone.commands.pop(cmd, None)
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
    while time.time() - start_time < timeout:
        if cmd in drone.commands and time.time() - drone.commands[cmd][0] < TIMEOUT:
            return drone.commands[cmd][1]
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
    start_time = time.time()
    while time.time() - start_time <= timeout:
        set_local_position(POSITION_ONLY, north, east, down, (0, 0, 0), (0, 0, 0), (0, 0))
        if reached_position(north, east, down, drone.x, drone.y, drone.z, tolerance):
            return True
        time.sleep(0.2)
    return False

def stop():
    msg = conn.mav.set_position_target_local_ned_encode(
            time_boot_ms=0,
            target_system=conn.target_system,
            target_component=conn.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,    # MAV_FRAME_BODY_NED
            type_mask=VELOCITY_ONLY,  # explicitly commanding only velocity to 0
            x=0.0, y=0.0, z=0.0,      
            vx=0, vy=0, vz=0, 
            afx=0.0, afy=0.0, afz=0.0,
            yaw=0.0, yaw_rate=0.0     
        )
    conn.mav.send(msg)
    return True

def return_to_base():
    msg = conn.mav.set_position_target_local_ned_encode(
            time_boot_ms=0,
            target_system=conn.target_system,
            target_component=conn.target_component,
            coordinate_frame=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  
            type_mask=0,           
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

def error_write(string):
    print(string)
    if ser and ser.is_open:
        ser.write(string.encode())

def ERROR(foo, *args):
    if foo(*args) == False:
        error_write(f"Error: Function {foo.__name__} failed")
        if foo.name != land.__name__:
            land(30)
        sys.exit()


# provide bitmask with conditions arranged by severity from LSB to MSB
def error_handle(bitmask, num_conditions):
    top_condition = bitmask & (0xF << (4 * (num_conditions - 1)))
    if top_condition & Fault.STALE_HEARTBEAT:
        error_write("STALE HEARTBEAT")
        land(30)
    elif top_condition & Fault.STALE_POSITION_LOST:
        error_write("STALE POSITION LOST")
    elif top_condition & Fault.STALE_GLOBAL_VELOCITY:
        error_write("STALE GLOBAL VELOCITY")
   
    
        


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
        if drone.system_fault & Fault.STALE_LANDED > 0:
           pass
        if drone.landed:
            return True
        time.sleep(0.2)
    return False

def wait_for_guided(timeout):
    start_time = time.time()
    while time.time() - start_time <= timeout:
        if drone.mode_guided == True:
            return True
        time.sleep(0.2)
    return False

def supervise():
    useful_messages = [
        "GPS_RAW_INT",
        "GLOBAL_POSITION_INT",
        "LOCAL_POSITION_NED",
        "HEARTBEAT",
        "EXTENDED_SYS_STATE",
        "SYS_STATUS",
        "COMMAND_ACK"   
    ]
    while 1:
        current_time = time.time()
        msg = conn.recv_match(type=useful_messages, blocking=True, timeout=0.2)
        if msg is not None:
            if msg.get_type() == "GPS_RAW_INT":
                drone.position_lost_timestamp = current_time
                drone.position_lost = msg.h_acc >= 3000
                if drone.position_lost and drone.position_lost_start:
                    drone.unknown_position_timer = current_time
                    drone.position_lost_start = 0
                if not drone.position_lost and not drone.position_lost_start:
                    drone.position_lost_start = 1
                drone.position_lost_timestamp = current_time
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                drone.relative_alt = msg.relative_alt / 1000.0
                drone.vx = msg.vx / 100.0
                drone.vy = msg.vy / 100.0
                drone.vz = msg.vz / 100.0
                drone.velocity_timestamp = current_time
            elif msg.get_type() == "LOCAL_POSITION_NED":
                drone.position_timestamp = current_time
                drone.x = msg.x
                drone.y = msg.y
                drone.z = msg.z
            elif msg.get_type() == "HEARTBEAT":
                drone.heartbeat_received = True
                drone.mode_guided = msg.custom_mode == GUIDED
                drone.armed = msg.base_mode & 128 == 128
                drone.heartbeat_timestamp = current_time
            elif msg.get_type() == "EXTENDED_SYS_STATE":
                drone.landed = msg.landed_state == 1 
                drone.landed_timestamp = current_time
            elif msg.get_type() == "SYS_STATUS":
                drone.battery_low = msg.battery_remaining < 30
                drone.battery_timestamp = current_time
            elif msg.get_type() == "COMMAND_ACK":
                drone.commands[msg.command] = current_time, msg.result
        if current_time - drone.heartbeat_timestamp > TIMEOUT:
            drone.system_fault |= Fault.STALE_HEARTBEAT     
        else:
            if not drone.armed:
                drone.system_fault |= Fault.FAULT_DISARMED
            else:
                drone.system_fault &= ~Fault.FAULT_DISARMED
            if not drone.mode_guided:
                drone.system_fault |= Fault.FAULT_NOT_GUIDED
            else:
                drone.system_fault &= ~Fault.FAULT_NOT_GUIDED
            drone.system_fault &= ~Fault.STALE_HEARTBEAT
        if current_time - drone.battery_timestamp > TIMEOUT:
            if drone.battery_low:
                # low battery
                drone.system_fault |= Fault.FAULT_BATTERY_LOW
            else:
                # stale data
                drone.system_fault |= Fault.STALE_BATTERY_LOW
        else:
            drone.system_fault &= ~(Fault.FAULT_BATTERY_LOW | Fault.STALE_BATTERY_LOW)
        if current_time - drone.unknown_position_timer > TIMEOUT:
            # lost position
            drone.system_fault |= Fault.FAULT_POSITION_LOST
        else:
            drone.system_fault &= ~Fault.FAULT_POSITION_LOST
        if drone.position_lost and current_time - drone.position_timestamp > TIMEOUT:
            # stale data
            drone.system_fault |= Fault.STALE_POSITION_LOST
        else:
            drone.system_fault &= ~Fault.STALE_POSITION_LOST
        if not drone.position_lost:
            drone.position_lost_start = 1
            drone.unknown_position_timer = 0
        time.sleep(0.2)


handler = {}
handler[Fault.STALE_HEARTBEAT] = [(stop, "Warning: STALE HEARTBEAT; Heartbeat not received recently")]
handler[Fault.STALE_BATTERY_LOW] = [(error_write, "Warning: STALE BATTERY; Battery level not fetched recently")]
handler[Fault.STALE_GLOBAL_VELOCITY] = stop
handler[Fault.STALE_LANDED] = stop
handler[Fault.STALE_LOCAL_POSITION] = stop
handler[Fault.STALE_POSITION_LOST] = land
handler[Fault.FAULT_BATTERY_LOW] = return_to_base


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
threading.Thread(target=supervise).start()

def main():
    conn.set_mode("GUIDED")
    ERROR(wait_for_guided, 10)
    ERROR(arm, 10)
    ERROR(takeoff, 10, 60, 1)
    ERROR(goto_local_ned, 5, 4, -10, 20)
    ERROR(relative_motion, 0.5, 0, 0, 2)
    ERROR(absolute_motion, 0.5, 0.2, 0, 4)
    ERROR(land, 45)

start_time = time.time()
while time.time() - start_time < 5:
    if drone.heartbeat_timestamp > 0:
        main()
        sys.exit(0)
    time.sleep(0.1)

sys.exit("Exiting main.py: No heartbeat received")
