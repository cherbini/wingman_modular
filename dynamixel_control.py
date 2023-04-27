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

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE = 24
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_PRESENT_POSITION = 36

        # Communication result
        self.COMM_SUCCESS = 0
        self.COMM_TX_FAIL = -1001

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

    def set_torque(self, servo_id, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MX_TORQUE_ENABLE, int(enable)
        )
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while enabling/disabling torque")

    def set_goal_position(self, servo_id, position):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MX_GOAL_POSITION, position
        )
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while setting goal position")

    def get_present_position(self, servo_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MX_PRESENT_POSITION
        )
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while reading present position")
        return dxl_present_position

    def close(self):
        # Disable Dynamixel torque
        self.set_torque(self.PAN_SERVO_ID, False)
        self.set_torque(self.TILT_SERVO_ID, False)

        # Close port
        self.portHandler.closePort()

    def servo_test(self):
        # Get initial position of PAN servo
        initial_position = self.get_present_position(self.PAN_SERVO_ID)

        # Move PAN servo +10 degrees from initial position
        self.set_goal_position(self.PAN_SERVO_ID, initial_position + 10)
        time.sleep(1)  # Wait for 1 second

        # Move PAN servo -10 degrees from initial position
        self.set_goal_position(self.PAN_SERVO_ID, initial_position - 10)
        time.sleep(1)  # Wait for 1 second

        # Move PAN servo back to initial position
        self.set_goal_position(self.PAN_SERVO_ID, initial_position)

# Usage example
if __name__ == "__main__":
    # Parameters: device port, baud rate, pan servo ID, tilt servo ID
    dynamixel_controller = DynamixelController("/dev/ttyDXL", 500000, 1, 2)

    # Set goal position for pan servo to 1000 and tilt servo to 2000
    dynamixel_controller.set_goal_position(dynamixel_controller.PAN_SERVO_ID, 1000)
    dynamixel_controller.set_goal_position(dynamixel_controller.TILT_SERVO_ID, 2000)
    # Read present position of pan and tilt servos
    pan_present_position = dynamixel_controller.get_present_position(dynamixel_controller.PAN_SERVO_ID)
    tilt_present_position = dynamixel_controller.get_present_position(dynamixel_controller.TILT_SERVO_ID)
    print("Pan servo present position:", pan_present_position)
    print("Tilt servo present position:", tilt_present_position)
    
    # Close and release resources
    dynamixel_controller.close()
    
