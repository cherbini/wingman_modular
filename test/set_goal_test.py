import time
from dynamixel_sdk import *

# Control table addresses for protocol 2.0
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_PAN_ID = 1  # Pan Dynamixel ID
DXL_TILT_ID = 2  # Tilt Dynamixel ID
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyDXL'  # Check which port is being used on your controller

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print('Succeeded to open the port')
else:
    print('Failed to open the port')
    exit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print('Succeeded to change the baudrate')
else:
    print('Failed to change the baudrate')
    exit()

# Enable Dynamixel torque
packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, 1)
packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, 1)

# Define the set_goal_position method
def set_goal_position(servo_id, goal_position):
    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_MX_GOAL_POSITION, goal_position)

# Define the square path for the servos
pan_offset = 1400  # Increase the offset for a larger square
tilt_offset = 500  # Increase the offset for a larger square
pan_positions = [2048, 2048 + pan_offset, 2048 + pan_offset, 2048 - pan_offset, 2048 - pan_offset, 2048]
tilt_positions = [2048, 2048, 2048 + tilt_offset, 2048 + tilt_offset, 2048 - tilt_offset, 2048]

# Move the servos in the square path
for pan_pos, tilt_pos in zip(pan_positions, tilt_positions):
    set_goal_position(DXL_PAN_ID, pan_pos)
    set_goal_position(DXL_TILT_ID, tilt_pos)
    time.sleep(1)  # Wait for 1 second for each move

# Disable Dynamixel torque
packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, 0)

# Close port
portHandler.closePort()

