import time
from dynamixel_sdk import *  # Dynamixel SDK library

# Control table addresses
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_PAN_ID = 1  # Pan Dynamixel ID
DXL_TILT_ID = 2  # Tilt Dynamixel ID
BAUDRATE = 500000
DEVICENAME = '/dev/ttyDXL'  # Check which port is being used on your controller

# Pan and tilt positions (in Dynamixel units)
PAN_CENTER = 2048
TILT_CENTER = 2048
PAN_LEFT = 1024
PAN_RIGHT = 3072
TILT_UP = 1024
TILT_DOWN = 3072

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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, 1)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, 1)

# Move pan and tilt to center position
packetHandler.write2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, PAN_CENTER)
packetHandler.write2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, TILT_CENTER)
time.sleep(1)

# Test pan movement
packetHandler.write2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, PAN_LEFT)
time.sleep(1)
packetHandler.write2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, PAN_RIGHT)
time.sleep(1)
packetHandler.write2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, PAN_CENTER)
time.sleep(1)

# Test tilt movement
packetHandler.write2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, TILT_UP)
time.sleep(1)
packetHandler.write2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, TILT_DOWN)
time.sleep(1)
packetHandler.write2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, TILT_CENTER)
time.sleep(1)

#Disable Dynamixel torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, 0)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, 0)

#Close port
portHandler.closePort()

