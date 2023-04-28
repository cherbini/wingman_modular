import dynamixel_sdk as sdk
import time

class DynamixelController:
    def __init__(self, device_port, baudrate, pan_servo_id, tilt_servo_id):
        # Protocol version
        self.PROTOCOL_VERSION = 2.0

        # Default setting
        self.DEVICE_PORT = device_port
        self.BAUDRATE = baudrate
        self.PAN_SERVO_ID = pan_servo_id
        self.TILT_SERVO_ID = tilt_servo_id

        # Control table address for Protocol 2.0 (MX series)
        self.ADDR_MX_TORQUE_ENABLE = 64
        self.ADDR_MX_GOAL_POSITION = 116
        self.LEN_GOAL_POSITION = 4  # Data Byte Length
        self.ADDR_MX_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4  # Data Byte Length

        # Communication result
        self.COMM_SUCCESS = sdk.COMM_SUCCESS
        self.COMM_TX_FAIL = sdk.COMM_TX_FAIL

        # Initialize PortHandler instance
        self.portHandler = sdk.PortHandler(self.DEVICE_PORT)

        # Initialize PacketHandler instance
        self.packetHandler = sdk.PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            raise Exception("Failed to open the Dynamixel port")

        # Set port baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise Exception("Failed to set the Dynamixel baudrate")

        # Enable Dynamixel torque
        self.set_torque(self.PAN_SERVO_ID, True)
        self.set_torque(self.TILT_SERVO_ID, True)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = sdk.GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_MX_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instance
        self.groupSyncRead = sdk.GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        self.groupSyncRead.addParam(self.PAN_SERVO_ID)
        self.groupSyncRead.addParam(self.TILT_SERVO_ID)

    def set_torque(self, servo_id, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MX_TORQUE_ENABLE, int(enable)
        )
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while enabling/disabling torque")

    def set_goal_position(self, pan_position, tilt_position):
        # Ensure pan_position and tilt_position are within valid range [0, 4095]
        pan_position = min(max(pan_position, 0), 4095)
        tilt_position = min(max(tilt_position, 0), 4095)

        # Allocate goal position values into byte array for pan servo
        param_goal_position_pan = [
            pan_position & 0xFF,
            (pan_position >> 8) & 0xFF,
            (pan_position >> 16) & 0xFF,
            (pan_position >> 24) & 0xFF
        ]

        # Allocate goal position values into byte array for tilt servo
        param_goal_position_tilt = [
            tilt_position & 0xFF,
            (tilt_position >> 8) & 0xFF,
            (tilt_position >> 16) & 0xFF,
            (tilt_position >> 24) & 0xFF
        ]

        # Add goal position values for pan and tilt servos
        self.groupSyncWrite.addParam(self.PAN_SERVO_ID, param_goal_position_pan)
        self.groupSyncWrite.addParam(self.TILT_SERVO_ID, param_goal_position_tilt)

        # Execute SyncWrite
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while setting goal position")

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def get_present_position(self):
        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while reading present position")

        # Get pan servo present position value
        pan_present_position = self.groupSyncRead.getData(self.PAN_SERVO_ID, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        # Get tilt servo present position value
        tilt_present_position = self.groupSyncRead.getData(self.TILT_SERVO_ID, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        return pan_present_position, tilt_present_position

    def close(self):
        # Disable Dynamixel torque
        self.set_torque(self.PAN_SERVO_ID, False)
        self.set_torque(self.TILT_SERVO_ID, False)

        # Close port
        self.portHandler.closePort()

    def servo_test(self):
        # Get initial positions of PAN and TILT servos (no additional arguments needed)
        pan_initial_position, tilt_initial_position = self.get_present_position()

        # Define the step size for the test (e.g., 10 ticks)
        step_size = 10

        # Move PAN servo +10 degrees from initial position (clamp to valid range)
        new_pan_position = min(max(pan_initial_position + step_size, 0), 4095)
        self.set_goal_position(new_pan_position, tilt_initial_position)
        time.sleep(1)  # Wait for 1 second

        # Move PAN servo -10 degrees from initial position (clamp to valid range)
        new_pan_position = min(max(pan_initial_position - step_size, 0), 4095)
        self.set_goal_position(new_pan_position, tilt_initial_position)
        time.sleep(1)  # Wait for 1 second

        # Move PAN servo back to initial position
        self.set_goal_position(pan_initial_position, tilt_initial_position)

# Usage example
if __name__ == "__main__":
    # Parameters: device port, baud rate, pan servo ID, tilt servo ID
    dynamixel_controller = DynamixelController("/dev/ttyDXL", 1000000, 1, 2)

    # Perform servo test
    dynamixel_controller.servo_test()
    
    # Close and release resources
    dynamixel_controller.close()
