"""
Encoder interface using SF12S03 encoders.

Blocking version.

Also handle the calibration
"""
import yaml
import os
import time
import numpy as np
from typing import Dict, Any, List  
from loguru import logger
from mini_ros.common.device import MotorDevice, MotorConfig, motor_config_from_json
from mini_ros.devices.motors.SF15S03 import SF15S03Encoder


class EncoderReader(MotorDevice):
    """
    Encoder interface using SF12S03 encoders.
    Reading encoder from one line is fast.
    Joint config, each item includes:
        - port: port name
        - id: device id
        - sign: sign of the joint
        - zero_point: zero point of the joint
        - gain: gain of the joint
        - ref_point: reference point of the joint
    """
    name = "encoder"

    def __init__(self):
        pass

    def initialize(self, joint_config: List[MotorConfig]):
        self.joint_config: List[MotorConfig] = joint_config
        
        # Build broadcast encoders and create mapping
        self.broadcast_encoders = {}  # port -> encoder
        self.port_to_joints = {}      # port -> list of joint_names
        self.joint_to_port_info = {}  # joint_name -> (port, device_id)
        self.joint_to_qpos_idx = {}
        
        self._setup_broadcast_encoders()
        self._setup_joint_mappings()
        self.num_joints = len(self.joint_config)
           
        logger.info(f"GelloEncoder initialized with broadcast encoders")
        logger.info(f"Encoder ports: {list(self.broadcast_encoders.keys())}")
        for port, joints in self.port_to_joints.items():
            logger.info(f"  {port}: {len(joints)} joints {joints}")

    def _setup_broadcast_encoders(self):
        """Group joints by port and create broadcast encoders"""
        logger.info("Setting up broadcast encoders...")
        
        # Group joints by port
        port_device_ids = {}  # port -> list of device_ids
        
        for config in self.joint_config:
            port = config.port
            device_id = config.id
            joint_name = config.joint_name
            
            if port not in port_device_ids:
                port_device_ids[port] = []
                self.port_to_joints[port] = []
            
            if device_id not in port_device_ids[port]:
                port_device_ids[port].append(device_id)
            
            self.port_to_joints[port].append(joint_name)
            self.joint_to_port_info[joint_name] = (port, device_id)
        
        # Create broadcast encoders for each port
        for port, device_ids in port_device_ids.items():
            try:
                device_ids_sorted = sorted(device_ids)
                
                if len(device_ids_sorted) == 1:
                    # Single encoder - use regular mode for compatibility
                    enc = SF15S03Encoder(port=port, device_id=device_ids_sorted[0],
                                       baudrate=1000000, timeout=0.01)
                else:
                    # Multiple encoders - use broadcast mode
                    enc = SF15S03Encoder(port=port, device_ids=device_ids_sorted,
                                       baudrate=1000000, timeout=0.01, strict_crc=False)
                
                self.broadcast_encoders[port] = enc
                logger.info(f"[Yes] Broadcast encoder initialized: {port} with IDs {device_ids_sorted}")
                
            except Exception as e:
                logger.error(f"[No] Failed to initialize broadcast encoder for {port}: {e}")
                raise
    
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

    def get_state(self):
        # Read all encoders using broadcast mode
        self.raw_joint_angles = {}
        qpos = np.zeros(self.num_joints)
        
        for port, encoder in self.broadcast_encoders.items():
            try:
                # Get broadcast data for this port
                data = encoder.get()
                angles = data["angle"]
                device_ids = data["device_ids"]
                crc_success = data["crc_success"]
                
                # Map results back to individual joints
                joints_on_port = self.port_to_joints[port]
                
                for joint_name in joints_on_port:
                    port_info, device_id = self.joint_to_port_info[joint_name]
                    
                    try:
                        # Find the index of this device_id in the results
                        device_index = device_ids.index(device_id)
                        
                        if isinstance(angles, np.ndarray) and device_index < len(angles):
                            raw_read = angles[device_index]
                            crc_ok = crc_success[device_index] if isinstance(crc_success, np.ndarray) else True
                            
                            if crc_ok:
                                self.raw_joint_angles[joint_name] = raw_read
                            else:
                                logger.warning(f"[Warning] CRC failure for {joint_name} (ID {device_id}) at {port}")
                                self.raw_joint_angles[joint_name] = None
                        else:
                            logger.warning(f"[Warning] Invalid angle data for {joint_name} (ID {device_id}) at {port}")
                            self.raw_joint_angles[joint_name] = None
                            
                    except (ValueError, IndexError) as e:
                        logger.warning(f"[Warning] Could not find device {device_id} for joint {joint_name}: {e} at {port}")
                        self.raw_joint_angles[joint_name] = None
                        
            except Exception as e:
                logger.warning(f"[Warning] Failed to read from port {port}: {e} at {port}")
                # Set all joints on this port to None
                for joint_name in self.port_to_joints[port]:
                    self.raw_joint_angles[joint_name] = None
        
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
        pass


if __name__ == "__main__":
    import time
    joint_map_file = "/home/haonan/Projects/miniROS/assets/gellos/joint_map_encoder.yaml"
    with open(joint_map_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    motor_config = motor_config_from_json(joint_config)
    encoder_driver = EncoderReader()
    encoder_driver.initialize(motor_config)
    while True:
        qpos = encoder_driver.get_state()
        logger.info(qpos)
        time.sleep(0.01)
    encoder_driver.stop()