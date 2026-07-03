from pymavlink import mavutil
import serial

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
        p1,
        p2, p3, p4, p5, p6, p7,
    )
    return conn.recv_match(type="COMMAND_ACK", blocking=True)

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
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Use body-fixed relative offset frame
        bitmask, 
        north,  # X: Position North (meters)
        east,  # Y: Position East (Negative = 5m West)
        down,   # Z: Position Down (0.0 to stay at current altitude)
        vel.x, vel.y, vel.z, # X, Y, Z velocity (ignored by bitmask)
        accel.x, accel.y, accel.z, # X, Y, Z acceleration (ignored by bitmask)
        yaw.target, yaw.rate     # Yaw target, Yaw rate (ignored by bitmask)
    )
    
def simple_goto(north, east, down):
    set_local_position(0b110111111000, north, east, down, 0, 0, 0)

def error_handle(foo, *args):
    print(foo(*args))

def arm():
    if send_command(mavutil.mavlink.MVD_CMD_COMPONENT_ARM_DISARM) == 0:
        return "ARM OK"
    else:
        return "ARM failed"

def takeoff(altitude):
    if send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=altitude) == 0:
        return "TAKEOFF OK"
    else:
        return "TAKEOFF failed"
    
def land():
    if send_command(mavutil.mavlink.MAV_CMD_NAV_LAND) == 0:
        return "LAND OK"
    else:
        return "LAND failed"
    

# jetson connected to FC via USB
conn = mavutil.mavlink_connection("/dev/ttyACM0")
conn.wait_heartbeat()

def run():
    conn.set_mode("GUIDED")
    arm()
    takeoff(10)
    simple_goto(5, 4, 2)
    land()

run()