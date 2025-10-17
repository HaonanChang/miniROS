"""
Fetch gello.
"""
from scservo_sdk import * 
import os
import yaml
import numpy as np
from typing import Dict, Any, List
from loguru import logger
from mini_ros.common.device import Reader, motor_config_from_json

DEBUG_GELLO_FEETCH = True


class FeetechReader(Reader):
    """
    Encoder interface using Feetech motors.
    Reading encoder from one line is fast.
    Joint config, each item includes:
        - port: port name
        - id: device id
        - sign: sign of the joint
        - zero_point: zero point of the joint
        - gain: gain of the joint
        - ref_point: reference point of the joint
    """
    name = "feetech"

    def __init__(self):
        pass

    def initialize(self, joint_config: List[Any]):
        self.ADDR_STS_PRESENT_POSITION  = 56
        self.BAUDRATE = 1000000           # SCServo default baudrate : 1000000
        self.protocol_end = 0
        self.joint_config = joint_config
        self.feetech_handlers = {}  # port -> (port_handler, packet_handler)
        self.joint_to_qpos_idx = {}
        # For feetech motor, different motors can't use one groupsync
        self.group_readers = {}       # port-motor_type -> GroupSyncRead objec
        self._setup_feetch_communication()
        self._setup_joint_mappings()

    def _setup_feetch_communication(self):
        # Group joints by port
        port_motor_device_ids = {}  # port_motor_type -> list of device_ids
        self.joint_to_port_motor_info = {}
        self.port_motor_info_to_joint = {}
        
        for joint_config in self.joint_config:
            port = joint_config.port
            device_id = joint_config.id
            motor_type = joint_config.motor_type
            port_motor = f"{port}_{motor_type}"
            joint_name = joint_config.joint_name
        
            if port_motor not in port_motor_device_ids:
                port_motor_device_ids[port_motor] = []
            
            if port_motor not in self.port_motor_info_to_joint:
                self.port_motor_info_to_joint[port_motor] = []
            
            port_motor_device_ids[port_motor].append(device_id)
            self.joint_to_port_motor_info[joint_name] = (port_motor, device_id)
            self.port_motor_info_to_joint[port_motor].append(joint_name)
        
        # Initialize Feetech handlers for each port
        for port_motor, device_ids in port_motor_device_ids.items():
            try:
                port, motor_type = port_motor.split("_")
                if port in self.feetech_handlers:
                    port_handler, packet_handler = self.feetech_handlers[port]
                else:
                    # Initialize port handler
                    port_handler = PortHandler(port)
                    if not port_handler.openPort():
                        raise Exception(f"Failed to open port {port}")
                    
                    # Set baudrate
                    if not port_handler.setBaudRate(self.BAUDRATE):
                        raise Exception(f"Failed to set baudrate {self.BAUDRATE} for port {port}")
                    
                    # Initialize packet handler
                    packet_handler = PacketHandler(self.protocol_end)
                    
                active_motors = []
                for device_id in sorted(device_ids):
                    # Get SCServo model number
                    scs_model_number, scs_comm_result, scs_error = packet_handler.ping(port_handler, device_id)
                    if scs_comm_result != COMM_SUCCESS:
                        logger.error("%s" % packet_handler.getTxRxResult(scs_comm_result))
                    elif scs_error != 0:
                        logger.error("%s" % packet_handler.getRxPacketError(scs_error))
                    else:
                        logger.info("[ID:%03d] ping Succeeded. SCServo model number : %d" % (device_id, scs_model_number))
                        active_motors.append(device_id)
                
                if active_motors:
                    if port not in self.feetech_handlers:
                        self.feetech_handlers[port] = (port_handler, packet_handler)
                    # Setup GroupSyncRead for faster bulk reading
                    group_sync_read = GroupSyncRead(port_handler, packet_handler, self.ADDR_STS_PRESENT_POSITION, 4)
                    for device_id in active_motors:
                        scs_addparam_result = group_sync_read.addParam(device_id)
                        if not scs_addparam_result:
                            logger.error(f"Failed to add parameter for device ID: {device_id}")
                    self.group_readers[port_motor] = group_sync_read
                else:
                    if port not in self.feetech_handlers:
                        port_handler.closePort()
                    logger.error(f"[Fail]: No active motors found on {port}")

            except Exception as e:
                logger.error(f"[Fail]: Failed to initialize Feetech communication for {port}: {e}")
                # Don't raise, allow operation without sensors
                pass

    def _setup_joint_mappings(self):
        """Setup joint name to qpos index mappings, sorted numerically by joint name"""
        
        # Sort joint names by the numerical part (e.g., joint_1, joint_2, ..., joint_10)
        sorted_joint_names = sorted(
            [joint_config.joint_name for joint_config in self.joint_config],
            key=lambda name: int(name.split('_')[-1])  # Extracts the number from 'joint_#'
        )

        # Create mapping
        joint_name_mapping = {
            name: i for i, name in enumerate(sorted_joint_names)
        }

        for joint_name in sorted_joint_names:
            self.joint_to_qpos_idx[joint_name] = joint_name_mapping[joint_name]
            logger.info(f"Mapping {joint_name} -> qpos[{joint_name_mapping[joint_name]}]")

    def get_gello_info(self):
        return self.joint_config

    def get_state(self):
        # Read all Feetech motors using fast GroupSyncRead
        self.raw_joint_angles = {}
        self.raw_joint_speeds = {}
        qpos = np.zeros(len(self.joint_to_qpos_idx))
        for port_motor in self.group_readers.keys():
            group_sync_read = self.group_readers[port_motor]
            joints_on_port = self.port_motor_info_to_joint[port_motor]
            
            # Single broadcast transaction for all motors on this port
            scs_comm_result = group_sync_read.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                logger.warning(f"[Warning]: GroupSyncRead communication failed on {port_motor}: {scs_comm_result}")
                continue
                # Set all joints on this port to None
            for joint_name in joints_on_port:
                device_id = self.joint_to_port_motor_info[joint_name][1]
                scs_getdata_result = group_sync_read.isAvailable(device_id, self.ADDR_STS_PRESENT_POSITION, 4)
                if scs_getdata_result == True:
                    positions_speed = group_sync_read.getData(device_id, self.ADDR_STS_PRESENT_POSITION, 4)
                    positions = SCS_LOWORD(positions_speed)
                    speed = SCS_HIWORD(positions_speed)
                # Handle negative
                if positions < (2 ** 15 - 1):
                    signed_positions = np.int16(positions)
                else:
                    signed_positions = -np.int16(positions - 2 ** 15)
                if speed < (2 ** 15 - 1):
                    signed_speed = np.int16(speed)
                else:
                    signed_speed = -np.int16(speed - 2 ** 15)
                # Convert positions to degrees
                angles_degrees = signed_positions * 360.0 / 4096.0
                # Convert speed to degrees per second
                speed_degrees_per_second = signed_speed * 360.0 / 4096.0

                self.raw_joint_angles[joint_name] = angles_degrees
                self.raw_joint_speeds[joint_name] = speed_degrees_per_second
        
        # Apply calibration corrections and update model
        for joint_config in self.joint_config:
            joint_name = joint_config.joint_name
            if joint_name in self.raw_joint_angles and self.raw_joint_angles[joint_name] is not None:
                corrected_read = self.calibrate_read(self.raw_joint_angles[joint_name], joint_config)
                qpos_idx = self.joint_to_qpos_idx[joint_name]
                qpos[qpos_idx] = corrected_read
            else:
                # Keep current position if sensor failed
                pass

        return qpos

    def stop(self):
        for port_handler, packet_handler in self.feetech_handlers.values():
            port_handler.closePort()



if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_map_file = os.path.join(root_dir, "..", "assets", "gellos", "joint_map_feetch.yaml")
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)

    motor_config = motor_config_from_json(joint_config)
    gello = FeetechReader()
    gello.initialize(motor_config)

    while True:
        gello.get_state()
        time.sleep(0.01)

    gello.stop()