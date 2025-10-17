"""
Provide a general simple gello interface.
"""

# Replace SF15S03 import with Dynamixel SDK
from dynamixel_sdk import *
from dynamixel_sdk import GroupSyncRead
import numpy as np
import yaml
import os
import time
from mini_ros.common.device import MotorDevice, MotorConfig, motor_config_from_json
from typing import List

from loguru import logger

DEBUG_DYNAMIXEL_GELLO = False


class DynamixelReader(MotorDevice):
    """
    Encoder interface using Dynamixel motors.
    Reading encoder from one line is fast.
    Joint config, each item includes:
        - port: port name
        - id: device id
        - sign: sign of the joint
        - zero_point: zero point of the joint
        - gain: gain of the joint
        - ref_point: reference point of the joint
    """
    name = "dynamixel"

    def __init__(self):
        pass

    def initialize(self, joint_config: List[MotorConfig]):
        self.joint_config = joint_config
        self.dynamixel_handlers = {}  # port -> (port_handler, packet_handler)
        self.port_to_joints = {}      # port -> list of joint_names
        self.joint_to_port_info = {}  # joint_name -> (port, device_id)
        self.joint_to_qpos_idx = {}
        self.group_readers = {}       # port -> GroupSyncRead objec
        self.num_joints = len(self.joint_config)

        # Dynamixel settings
        self.PROTOCOL_VERSION = 2.0
        self.BAUDRATE = 1000000
        self.TIMEOUT_MS = 10
        
        # XL330 Control Table addresses
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4
        self.ADDR_RETURN_DELAY_TIME = 9

        self._setup_dynamixel_communication()
        self._setup_joint_mappings()
        
    def _setup_dynamixel_communication(self):
        """Initialize Dynamixel communication for each port"""
        logger.info("Setting up Dynamixel communication...")
        
        # Group joints by port
        port_device_ids = {}  # port -> list of device_ids
        
        for joint_config in self.joint_config:
            port = joint_config.port
            device_id = joint_config.id
            
            if port not in port_device_ids:
                port_device_ids[port] = []
                self.port_to_joints[port] = []
            
            if device_id not in port_device_ids[port]:
                port_device_ids[port].append(device_id)
            
            self.port_to_joints[port].append(joint_config.joint_name)
            self.joint_to_port_info[joint_config.joint_name] = (port, device_id)
        
        # Initialize Dynamixel handlers for each port
        for port, device_ids in port_device_ids.items():
            try:
                # Initialize port handler
                port_handler = PortHandler(port)
                if not port_handler.openPort():
                    raise Exception(f"Failed to open port {port}")
                
                # Set baudrate
                if not port_handler.setBaudRate(self.BAUDRATE):
                    raise Exception(f"Failed to set baudrate {self.BAUDRATE} for port {port}")
                
                # Initialize packet handler
                packet_handler = PacketHandler(self.PROTOCOL_VERSION)
                
                # Test communication with each motor
                active_motors = []
                for device_id in sorted(device_ids):
                    model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, device_id)
                    if dxl_comm_result == COMM_SUCCESS:
                        active_motors.append(device_id)
                        logger.info(f"[Succ]: Motor ID {device_id} found on {port} (Model: 0x{model_number:04X})")
                    else:
                        logger.info(f"[Fail]: Motor ID {device_id} not responding on {port}")
                        logger.info(f"  Communication result: {dxl_comm_result}, Error: {dxl_error}")
                
                if active_motors:
                    self.dynamixel_handlers[port] = (port_handler, packet_handler)
                    
                    # Setup GroupSyncRead for faster bulk reading
                    group_sync_read = GroupSyncRead(port_handler, packet_handler,
                                                   self.ADDR_PRESENT_POSITION,
                                                   self.LEN_PRESENT_POSITION)
                    for device_id in active_motors:
                        group_sync_read.addParam(device_id)
                        
                        # Set Return Delay Time to 0 for faster communication
                        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
                            port_handler, device_id, self.ADDR_RETURN_DELAY_TIME, 0)
                        if dxl_comm_result == COMM_SUCCESS:
                            print(f"[Succ]: Set Return Delay Time = 0 for motor ID {device_id}")
                        else:
                            print(f"[Warning]: Failed to set Return Delay Time for motor ID {device_id}")
                    
                    self.group_readers[port] = group_sync_read
                    logger.info(f"[Succ]: Dynamixel handler initialized for {port} with {len(active_motors)} motors")
                    logger.info(f"[Succ]: GroupSyncRead configured for {port} with {len(active_motors)} motors")
                else:
                    port_handler.closePort()
                    logger.info(f"[Fail]: No active motors found on {port}")
                    
            except Exception as e:
                logger.info(f"[Fail]: Failed to initialize Dynamixel communication for {port}: {e}")
                # Don't raise, allow operation without sensors
                pass
        
    def _setup_joint_mappings(self):
        """Setup joint name to qpos index mappings"""
        joint_name_mapping = {
            name: i for i, name in enumerate([joint_config.joint_name for joint_config in self.joint_config])
        }
        for joint_name in [joint_config.joint_name for joint_config in self.joint_config]:
            if joint_name in joint_name_mapping:
                self.joint_to_qpos_idx[joint_name] = joint_name_mapping[joint_name]
                logger.info(f"Mapping {joint_name} -> qpos[{joint_name_mapping[joint_name]}]")

    def get_gello_info(self):
        return self.joint_config
    
    def get_state(self):
        # Read all Dynamixel motors using fast GroupSyncRead
        all_joint_angles = {}
        qpos = np.zeros(self.num_joints)
        
        for port, (port_handler, packet_handler) in self.dynamixel_handlers.items():
            group_sync_read = self.group_readers[port]
            joints_on_port = self.port_to_joints[port]
            
            # Single broadcast transaction for all motors on this port
            dxl_comm_result = group_sync_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                logger.info(f"[Warning]: GroupSyncRead communication failed on {port}: {dxl_comm_result}")
                # Set all joints on this port to None
                for joint_name in joints_on_port:
                    all_joint_angles[joint_name] = None
                continue
            
            # Extract data for each joint
            if DEBUG_DYNAMIXEL_GELLO:
                logger.info("--------------------------------")
            for joint_name in joints_on_port:
                _, device_id = self.joint_to_port_info[joint_name]
                
                try:
                    # Check if data is available for this motor
                    if not group_sync_read.isAvailable(device_id, 
                                                       self.ADDR_PRESENT_POSITION,
                                                       self.LEN_PRESENT_POSITION):
                        all_joint_angles[joint_name] = None
                        continue
                    
                    # Get the position data
                    dxl_present_position = group_sync_read.getData(device_id,
                                                                   self.ADDR_PRESENT_POSITION,
                                                                   self.LEN_PRESENT_POSITION)
                    
                    # Convert position to degrees (XL330: 4096 counts = 360°)
                    # Properly handle unsigned 32-bit to signed 32-bit conversion
                    if dxl_present_position < (2 ** 31 - 1):
                        signed_position = np.int32(dxl_present_position)
                    else:
                        signed_position = np.int32(dxl_present_position - 2 ** 32)
                    angle_degrees = signed_position * 360.0 / 4096
                    if DEBUG_DYNAMIXEL_GELLO:
                        logger.info(f"Joint {joint_name} angle: {angle_degrees}, signed_position: {signed_position}")
                    all_joint_angles[joint_name] = angle_degrees
                    
                except Exception as e:
                    logger.info(f"[Warning]: Exception extracting data for {joint_name} (ID {device_id}): {e}")
                    all_joint_angles[joint_name] = None
        
        # Apply calibration corrections and update model (focus on right arm)
        for joint_config in self.joint_config:
            joint_name = joint_config.joint_name
            if joint_name in all_joint_angles and all_joint_angles[joint_name] is not None:
                corrected_read = self.calibrate_read(all_joint_angles[joint_name], joint_config)
                qpos_idx = self.joint_to_qpos_idx[joint_name]
                qpos[qpos_idx] = corrected_read
            else:
                pass
        
        return qpos
    
    def stop(self):
        for port_handler, packet_handler in self.dynamixel_handlers.values():
            port_handler.closePort()


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_map_file = "/home/haonan/Projects/miniROS/assets/gellos/gello.yaml"
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)

    action_rate = 60 # Hz
    action_period = 1 / action_rate
    gello = DynamixelReader()

    gello.initialize(motor_config_from_json(joint_config))
    while True:
        qpos = gello.get_state()
        logger.info(qpos)
        time.sleep(action_period)
    gello.stop()