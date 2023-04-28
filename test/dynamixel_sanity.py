import os
import sys
import termios
import tty
import time
from dynamixel_sdk import *

# Control table addresses for protocol 2.0
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PRESENT_POSITION = 132
ADDR_MX_PRESENT_TEMPERATURE = 146
ADDR_MX_PRESENT_VOLTAGE = 144
ADDR_MX_PRESENT_CURRENT = 126

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_PAN_ID = 1  # Pan Dynamixel ID
DXL_TILT_ID = 2  # Tilt Dynamixel ID
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyDXL'  # Check which port is being used on your controller

# Pan and tilt positions (in Dynamixel units)
PAN_CENTER = 2048
TILT_CENTER = 2048
PAN_LEFT = 1024
PAN_RIGHT = 3072
TILT_UP = 1024
TILT_DOWN = 3072
STEP_SIZE = 50

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
torque_enabled = True

# Move pan and tilt to center position
packetHandler.write4ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, PAN_CENTER)
packetHandler.write4ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, TILT_CENTER)
time.sleep(1)

# Function to get user input (cross-platform)
def getch():
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode()
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Enter loop to continuously read and output data
while True:
    # Read and print pan servo data
    pan_position = packetHandler.read4ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_PRESENT_POSITION)[0]

    pan_temperature = packetHandler.read1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_PRESENT_TEMPERATURE)[0]
    pan_voltage = packetHandler.read2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_PRESENT_VOLTAGE)[0] / 1000.0  # Convert to volts
    pan_current = packetHandler.read2ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_PRESENT_CURRENT)[0]  # Convert to mA
    pan_torque_enabled = packetHandler.read1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE)[0] == 1  # Torque state

    # Read and print tilt servo data
    tilt_position = packetHandler.read4ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_PRESENT_POSITION)[0]
    tilt_temperature = packetHandler.read1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_PRESENT_TEMPERATURE)[0]
    tilt_voltage = packetHandler.read2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_PRESENT_VOLTAGE)[0] / 1000.0  # Convert to volts
    tilt_current = packetHandler.read2ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_PRESENT_CURRENT)[0]  # Convert to mA
    tilt_torque_enabled = packetHandler.read1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE)[0] == 1  # Torque state

    # Print data
    print("Pan Servo [ID:{}] - Position: {}, Temperature: {}°C, Voltage: {:.2f}V, Current: {}mA".format(
        DXL_PAN_ID, pan_position, pan_temperature, pan_voltage, pan_current))
    print("Tilt Servo [ID:{}] - Position: {}, Temperature: {}°C, Voltage: {:.2f}V, Current: {}mA".format(
        DXL_TILT_ID, tilt_position, tilt_temperature, tilt_voltage, tilt_current))
    print("Pan Servo [ID:{}] - Position: {}, Temperature: {}°C, Voltage: {:.2f}V, Current: {}mA, Torque: {}".format(
        DXL_PAN_ID, pan_position, pan_temperature, pan_voltage, pan_current, "Enabled" if pan_torque_enabled else "Disabled"))
    print("Tilt Servo [ID:{}] - Position: {}, Temperature: {}°C, Voltage: {:.2f}V, Current: {}mA, Torque: {}".format(
        DXL_TILT_ID, tilt_position, tilt_temperature, tilt_voltage, tilt_current, "Enabled" if tilt_torque_enabled else "Disabled"))

    # Check for user input to exit the loop
    print("Press e to toggle torque, arrow keys to control, ESC to exit...")
    print("Press ESC to exit, any other key to continue...")

        # Get user input
    key = getch()

    # Check for 'e' key to enable/disable torque
    if key == 'e':
        torque_enabled = not torque_enabled
        packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, int(torque_enabled))
        packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, int(torque_enabled))

    # Check for arrow keys to control pan and tilt
    elif key == '\x1b':
        # Read the next two characters (arrow key escape sequence)
        key += sys.stdin.read(2)

        # Arrow up - tilt up
        if key == '\x1b[A':
            tilt_position = max(tilt_position - STEP_SIZE, TILT_UP)
        # Arrow down - tilt down
        elif key == '\x1b[B':
            tilt_position = min(tilt_position + STEP_SIZE, TILT_DOWN)
        # Arrow left - pan left
        elif key == '\x1b[D':
            pan_position = max(pan_position - STEP_SIZE, PAN_LEFT)
        # Arrow right - pan right
        elif key == '\x1b[C':
            pan_position = min(pan_position + STEP_SIZE, PAN_RIGHT)
        # Update servo positions
        packetHandler.write4ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_GOAL_POSITION, pan_position)
        packetHandler.write4ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_GOAL_POSITION, tilt_position)

    # Check for user input to exit the loop (ESC key)
    elif key == chr(0x1b):
        break

# Disable Dynamixel torque
packetHandler.write1ByteTxRx(portHandler, DXL_PAN_ID, ADDR_MX_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_TILT_ID, ADDR_MX_TORQUE_ENABLE, 0)

# Close port
portHandler.closePort()
