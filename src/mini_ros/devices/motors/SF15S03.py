import serial                # <-- make sure this line is present
import struct
import time
import glob
import os
from typing import List, Dict, Union, Tuple
import numpy as np


class SF15S03Encoder:
    """Class to communicate with SF15S03 MODBUS RTU encoder (broadcast-first)."""
    
    # Register addresses (in decimal)
    REG_ANGLE_HIGH = 64      # 0x40
    REG_ANGLE_LOW = 65       # 0x41
    REG_SPEED_HIGH = 66      # 0x42
    REG_SPEED_LOW = 67       # 0x43
    REG_CIRCLE_COUNT = 68    # 0x44
    REG_MULTI_POS_HIGH = 70  # 0x46
    REG_MULTI_POS_LOW = 71   # 0x47
    
    def __init__(self, port, device_id=None, device_ids=None, baudrate=1000000, timeout=0.01, resolution=4096, strict_crc=False):
        """
        Initialize the encoder communication
        
        Args:
            port: Serial port name (e.g., 'COM1' or '/dev/ttyUSB0')
            device_id: Optional single MODBUS device ID (used for sequential diagnostic only)
            device_ids: List of MODBUS device IDs for broadcast functionality (default: [1..7])
            baudrate: Communication speed (default: 1000000)
            timeout: Read timeout in seconds
            resolution: Encoder resolution (default: 4096, common values: 1024, 4096, 65536)
            strict_crc: Whether to drop frames on CRC errors (default: False for robustness)
        """
        # Broadcast-first: assume 7 devices [1..7] unless explicit list is provided
        if device_ids is not None and len(device_ids) > 0:
            self.device_ids = list(device_ids)
        elif device_id is not None:
            # Maintain ability to talk to a single device for diagnostics
            self.device_ids = [device_id]
        else:
            self.device_ids = [1, 2, 3, 4, 5, 6, 7]
        self.device_id = self.device_ids[0]
        self._num = len(self.device_ids)
            
        self.resolution = resolution
        self.strict_crc = strict_crc
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        
        # Config for broadcast functionality
        self.angle_dtype = np.dtype(np.float64)
        self.angle_shape = (self._num,)
        
    def close(self):
        """Close serial port and release resources"""
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            print(f"Warning: Error closing serial port: {e}")
    
    def __del__(self):
        """Close serial port when object is destroyed"""
        self.close()
    
    def calculate_crc16_modbus(self, data):
        """
        Calculate MODBUS CRC16
        CRC-16/Modbus polynomial: x^16 + x^15 + x^2 + 1 (0xA001)
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def build_read_command(self, start_address, num_registers, device_id=None):
        """
        Build MODBUS RTU read command (function code 03)
        
        Args:
            start_address: Starting register address
            num_registers: Number of registers to read
            device_id: Device ID to use (if None, uses self.device_id)
            
        Returns:
            Complete command with CRC
        """
        if device_id is None:
            device_id = self.device_id
            
        command = bytearray()
        command.append(device_id)  # Device ID
        command.append(0x03)       # Function code: Read Holding Registers
        command.extend(struct.pack('>H', start_address))  # Start address (big-endian)
        command.extend(struct.pack('>H', num_registers))  # Number of registers
        
        # Calculate and append CRC
        crc = self.calculate_crc16_modbus(command)
        command.extend(struct.pack('<H', crc))  # CRC (little-endian)
        
        return command

    def build_broadcast_command(self, start_address, num_registers):
        """
        Build broadcast command similar to angler_broadcast.py
        Uses device ID 0x00 for broadcast
        
        Args:
            start_address: Starting register address
            num_registers: Number of registers to read
            
        Returns:
            Complete command with CRC as bytes
        """
        # Build command string like angler_broadcast.py: "00 03 00 41 00 01"
        command_hex = f"00 03 {start_address:04X} {num_registers:04X}"
        command_bytes = bytes.fromhex(command_hex.replace(' ', ''))
        
        # Calculate CRC
        crc = self.calculate_crc16_modbus(command_bytes)
        crc_H = (crc >> 8) & 0xFF
        crc_L = crc & 0xFF
        
        # Append CRC (low byte first, then high byte like angler_broadcast.py)
        command_bytes += bytes([crc_L, crc_H])
        
        return command_bytes

    def parse_response(self, response, verbose=False):
        """
        Parse MODBUS RTU response
        
        Args:
            response: Raw response bytes
            verbose: Print debug messages
            
        Returns:
            List of register values or None if error
        """
        if len(response) < 5:
            if verbose:
                print("Response too short")
            return None
            
        # Check device ID (allow any device ID for broadcast responses)
        device_id = response[0]
        if device_id not in self.device_ids and device_id != 0:
            if verbose:
                print(f"Device ID {device_id} not in expected list: {self.device_ids}")
            return None
            
        # Check function code
        if response[1] == 0x83:  # Error response
            if verbose:
                print(f"MODBUS error: {response[2]}")
            return None
        elif response[1] != 0x03:
            if verbose:
                print(f"Unexpected function code: {response[1]}")
            return None
            
        # Get byte count
        byte_count = response[2]
        
        # Check response length
        expected_length = 3 + byte_count + 2  # ID + FC + BC + data + CRC
        if len(response) != expected_length:
            if verbose:
                print(f"Response length mismatch: expected {expected_length}, got {len(response)}")
            return None
            
        # Verify CRC
        data_without_crc = response[:-2]
        received_crc = struct.unpack('<H', response[-2:])[0]
        calculated_crc = self.calculate_crc16_modbus(data_without_crc)
        
        if received_crc != calculated_crc:
            if verbose:
                print(f"CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}")
            if self.strict_crc:
                return None
            
        # Extract register values
        registers = []
        for i in range(0, byte_count, 2):
            value = struct.unpack('>H', response[3+i:3+i+2])[0]
            registers.append(value)
            
        return registers, device_id

    def parse_broadcast_response(self, response, verbose=False):
        """
        Parse broadcast response similar to angler_broadcast.py
        Each encoder returns 7 bytes: [device_id, func_code, byte_count, data_high, data_low, crc_low, crc_high]
        
        Args:
            response: Raw response bytes (7 * num_encoders)
            verbose: Print debug messages
            
        Returns:
            Dictionary with angles and metadata
        """
        if len(response) != 7 * self._num:
            if verbose:
                print(f"Broadcast response length mismatch: expected {7 * self._num}, got {len(response)}")
            return None
            
        not_received = set(self.device_ids)
        ret = np.zeros(self._num)
        crc_success = np.ones_like(ret)
        receive_time = time.time() * 1000
        
        for i in range(self._num):
            resp_slice = response[7*i:7*(i+1)]
            device_id = resp_slice[0]
            
            if device_id not in not_received:
                if verbose:
                    print(f"Unexpected device ID {device_id} or duplicate response")
                continue
                
            if resp_slice[1] != 3:  # Function code
                if verbose:
                    print(f"Unexpected function code {resp_slice[1]} from device {device_id}")
                continue
                
            if resp_slice[2] != 2:  # Byte count
                if verbose:
                    print(f"Unexpected byte count {resp_slice[2]} from device {device_id}")
                continue
            
            # Check CRC similar to angler_broadcast.py
            crc_data = resp_slice[:5]
            crc = self.calculate_crc16_modbus(crc_data)
            crc_H = (crc >> 8) & 0xFF
            crc_L = crc & 0xFF
            actual_crc_L = resp_slice[5]
            actual_crc_H = resp_slice[6]
            
            if not (crc_L == actual_crc_L and crc_H == actual_crc_H):
                crc_string = ' '.join([f'{x:02x}' for x in crc_data])
                print(f"Warning: Encoder {device_id} CRC error: expected {crc_L:02x} {crc_H:02x}, got {actual_crc_L:02x} {actual_crc_H:02x}, CRC String: {crc_string}")
                if self.strict_crc:
                    return self.read_angles_broadcast()  # Retry reading
                else:
                    crc_success[i] = 0
            
            not_received.remove(device_id)
            
            # Extract angle (16-bit value from high and low bytes)
            angle_raw = (resp_slice[3] << 8) | resp_slice[4]
            angle_degrees = 360.0 * angle_raw / self.resolution
            
            # Map to correct position in result array
            device_index = self.device_ids.index(device_id)
            ret[device_index] = angle_degrees
            
        return {
            'angle': ret, 
            'timestamp_ms': receive_time, 
            'crc_success': crc_success,
            'device_ids': self.device_ids
        }

    def detect_resolution(self):
        """
        Auto-detect encoder resolution by reading multiple samples
        and finding the maximum single-turn value
        """
        print("Auto-detecting encoder resolution...")
        max_value = 0
        
        # Read 10 samples over 2 seconds
        for i in range(10):
            command = self.build_read_command(0x40, 2)
            self.ser.reset_input_buffer()
            self.ser.write(command)
            response = self.ser.read(9)
            
            result = self.parse_response(response, verbose=True)
            if result:
                registers, _ = result
                angle_raw = (registers[0] << 16) | registers[1]
                single_turn = angle_raw & 0xFFFF  # Lower 16 bits
                max_value = max(max_value, single_turn)
            
            time.sleep(0.2)
        
        # Common resolutions and their expected max values
        resolutions = [
            (64, "6-bit"),
            (256, "8-bit"),
            (512, "9-bit"),
            (1024, "10-bit"),
            (4096, "12-bit"),
            (32768, "15-bit"),
            (65536, "16-bit")
        ]
        
        # Find the most likely resolution
        # Max value should be close to but less than the resolution
        detected_resolution = 4096  # Default
        
        for res, desc in resolutions:
            if max_value < res:
                detected_resolution = res
                print(f"Detected resolution: {res} ({desc})")
                break
        
        return detected_resolution

    def read_angles_broadcast(self) -> Dict[str, Union[np.ndarray, float]]:
        """
        Read angles from multiple encoders using broadcast method similar to angler_broadcast.py
        
        Returns:
            Dictionary with angle data similar to angler_broadcast.py format
        """
        # Ensure no data in buffer
        if self.ser.in_waiting > 0:
            self.ser.reset_input_buffer()

        # Send broadcast command to read angle register (0x41 = 65 decimal)
        command = self.build_broadcast_command(0x0041, 0x0001)
        self.ser.write(command)

        # Collect all available 7-byte frames within a short window
        # This avoids leaving extra frames in the buffer when more devices
        # respond than self._num (e.g., instance constructed with a single ID)
        receive_time = time.time() * 1000
        buffer = bytearray()

        # Initial wait window in seconds
        quiet_deadline = time.time() + 0.02
        while True:
            time.sleep(0.001)
            waiting = self.ser.in_waiting
            if waiting:
                buffer += self.ser.read(waiting)
                # Extend the quiet deadline a bit to catch stragglers
                quiet_deadline = time.time() + 0.002
            if time.time() > quiet_deadline:
                break

        # Parse frames in chunks of 7 bytes
        num_frames = len(buffer) // 7
        # Prepare outputs for the configured device_ids
        angles = np.zeros(self._num)
        crc_success = np.zeros(self._num)
        not_received = set(self.device_ids)

        for i in range(num_frames):
            resp_slice = buffer[7*i:7*(i+1)]
            if len(resp_slice) != 7:
                continue

            device_id = resp_slice[0]
            # Ignore devices we did not configure, but still consume their frames
            if device_id not in self.device_ids:
                continue

            # Validate func and byte count
            if resp_slice[1] != 3 or resp_slice[2] != 2:
                continue

            # CRC check
            crc_data = resp_slice[:5]
            crc = self.calculate_crc16_modbus(crc_data)
            crc_H = (crc >> 8) & 0xFF
            crc_L = crc & 0xFF
            actual_crc_L = resp_slice[5]
            actual_crc_H = resp_slice[6]
            if not (crc_L == actual_crc_L and crc_H == actual_crc_H):
                if self.strict_crc:
                    # Skip this frame; caller can retry by calling again
                    continue
                else:
                    # Mark CRC failure but still parse the value
                    pass

            # Decode angle
            angle_raw = (resp_slice[3] << 8) | resp_slice[4]
            angle_degrees = 360.0 * angle_raw / self.resolution

            # Map to index and store
            device_index = self.device_ids.index(device_id)
            if device_id in not_received:
                angles[device_index] = angle_degrees
                # Only mark success if CRC matched, otherwise leave 0
                if (crc_L == actual_crc_L and crc_H == actual_crc_H):
                    crc_success[device_index] = 1.0
                not_received.remove(device_id)

        return {
            'angle': angles,
            'timestamp_ms': receive_time,
            'crc_success': crc_success,
            'device_ids': self.device_ids
        }

    def read_angles_broadcast_discover(self) -> Dict[str, Union[np.ndarray, float]]:
        """
        Broadcast read that discovers ALL responding devices, without requiring
        them to be listed in self.device_ids. Useful for dynamic discovery.

        Returns a dict with keys:
          - 'angle': np.ndarray of angles in degrees, aligned with 'device_ids'
          - 'timestamp_ms': float
          - 'crc_success': np.ndarray of 1/0 per device
          - 'device_ids': list[int] of discovered device IDs (sorted)
        """
        # Flush input
        if self.ser.in_waiting > 0:
            self.ser.reset_input_buffer()

        # Send broadcast command
        command = self.build_broadcast_command(0x0041, 0x0001)
        self.ser.write(command)

        receive_time = time.time() * 1000
        buffer = bytearray()

        # Collect frames for a short window
        quiet_deadline = time.time() + 0.02
        while True:
            time.sleep(0.001)
            waiting = self.ser.in_waiting
            if waiting:
                buffer += self.ser.read(waiting)
                quiet_deadline = time.time() + 0.002
            if time.time() > quiet_deadline:
                break

        # Parse all 7-byte frames and accumulate by device ID
        num_frames = len(buffer) // 7
        discovered: Dict[int, Tuple[float, float]] = {}
        # store (angle_deg, crc_ok)

        for i in range(num_frames):
            resp_slice = buffer[7*i:7*(i+1)]
            if len(resp_slice) != 7:
                continue
            device_id = resp_slice[0]
            # func and byte count check
            if resp_slice[1] != 3 or resp_slice[2] != 2:
                continue
            # CRC check
            crc_data = resp_slice[:5]
            crc = self.calculate_crc16_modbus(crc_data)
            crc_H = (crc >> 8) & 0xFF
            crc_L = crc & 0xFF
            actual_crc_L = resp_slice[5]
            actual_crc_H = resp_slice[6]
            crc_ok = (crc_L == actual_crc_L and crc_H == actual_crc_H)

            angle_raw = (resp_slice[3] << 8) | resp_slice[4]
            angle_degrees = 360.0 * angle_raw / self.resolution
            # Keep the last valid frame per device
            discovered[device_id] = (angle_degrees, 1.0 if crc_ok else 0.0)

        # Build outputs
        if not discovered:
            return {
                'angle': np.zeros(0),
                'timestamp_ms': receive_time,
                'crc_success': np.zeros(0),
                'device_ids': []
            }

        device_ids_sorted = sorted(discovered.keys())
        angles = np.array([discovered[d][0] for d in device_ids_sorted], dtype=np.float64)
        crc_success = np.array([discovered[d][1] for d in device_ids_sorted], dtype=np.float64)

        return {
            'angle': angles,
            'timestamp_ms': receive_time,
            'crc_success': crc_success,
            'device_ids': device_ids_sorted
        }

    def read_angles_sequential(self) -> Dict[str, Union[np.ndarray, float]]:
        """
        Read angles from multiple encoders using sequential MODBUS commands
        This is more reliable than broadcast for standard MODBUS devices
        
        Returns:
            Dictionary with angle data in angler_broadcast.py format
        """
        angles = np.zeros(self._num)
        crc_success = np.ones(self._num)
        receive_time = time.time() * 1000
        
        for i, device_id in enumerate(self.device_ids):
            try:
                command = self.build_read_command(0x40, 2, device_id)
                self.ser.reset_input_buffer()
                self.ser.write(command)
                response = self.ser.read(9)  # ID + FC + BC + 4 bytes + CRC
                
                result = self.parse_response(response)
                if result:
                    registers, resp_device_id = result
                    if len(registers) >= 2:
                        angle_raw = (registers[0] << 16) | registers[1]
                        single_turn = angle_raw % self.resolution
                        angles[i] = single_turn * 360.0 / self.resolution
                    else:
                        crc_success[i] = 0
                        print(f"Warning: Insufficient data from device {device_id}")
                else:
                    crc_success[i] = 0
                    print(f"Warning: Failed to read from device {device_id}")
                    
            except Exception as e:
                crc_success[i] = 0
                print(f"Warning: Exception reading from device {device_id}: {e}")
        
        return {
            'angle': angles,
            'timestamp_ms': receive_time,
            'crc_success': crc_success,
            'device_ids': self.device_ids
        }

    def get(self) -> Dict[str, Union[np.ndarray, float]]:
        """
        Get angle data from all configured encoders via broadcast.
        Always uses broadcast and never falls back to sequential.
        """
        return self.read_angles_broadcast()

    def read_all_data(self):
        """
        Read all encoder data in one request
        
        Returns:
            Dictionary with encoder data or None if error
        """
        # Auto-detect resolution if not set
        if self.resolution is None:
            self.resolution = self.detect_resolution()
            
        # Try to read 8 registers starting from 0x40 (64)
        command = self.build_read_command(0x40, 8)
        
        # Clear input buffer
        self.ser.reset_input_buffer()
        
        # Send command (only show debug in verbose mode)
        self.ser.write(command)
        
        # Read response
        # Expected: ID(1) + FC(1) + BC(1) + Data(16) + CRC(2) = 21 bytes
        response = self.ser.read(21)
        
        # Parse response
        result = self.parse_response(response)
        if result is None:
            # Fallback: try reading just angle data
            return self._read_basic_data()
        registers, _ = result
        if len(registers) < 2:
            # Fallback: try reading just angle data
            return self._read_basic_data()
            
        # Extract values based on available registers
        data = {}
        
        # Angle data (always available if we got here)
        if len(registers) >= 2:
            data['angle_raw'] = (registers[0] << 16) | registers[1]  # 32-bit angle
            data['single_turn_value'] = data['angle_raw'] % self.resolution
            data['angle_degrees'] = data['single_turn_value'] * 360.0 / self.resolution
        
        # Speed data (optional)
        if len(registers) >= 4:
            data['speed_raw'] = (registers[2] << 16) | registers[3]  # 32-bit speed (signed)
            # Convert speed (signed 32-bit)
            if data['speed_raw'] > 0x7FFFFFFF:
                data['speed_raw'] = data['speed_raw'] - 0x100000000
            data['speed_rpm'] = data['speed_raw'] / 100.0  # Speed register value / 100
        else:
            data['speed_raw'] = 0
            data['speed_rpm'] = 0.0
        
        # Circle count (optional)
        if len(registers) >= 5:
            data['circle_count'] = registers[4]
        else:
            data['circle_count'] = 0
        
        # Multi-turn position (optional)
        if len(registers) >= 8:
            data['multi_turn_position'] = (registers[6] << 16) | registers[7]  # 32-bit position
            data['encoder_value'] = data['multi_turn_position']
        else:
            # Calculate approximate multi-turn position from circle count and angle
            data['multi_turn_position'] = data['circle_count'] * self.resolution + data['single_turn_value']
            data['encoder_value'] = data['multi_turn_position']
        
        return data
    
    def _read_basic_data(self):
        """
        Fallback method to read basic angle data when full data read fails
        """
        # Read just the angle registers
        command = self.build_read_command(0x40, 2)
        self.ser.reset_input_buffer()
        self.ser.write(command)
        response = self.ser.read(9)  # ID + FC + BC + 4 bytes + CRC
        
        result = self.parse_response(response)
        if result is None:
            return None
        registers, _ = result
        if len(registers) < 2:
            return None
            
        # Extract basic angle data
        angle_raw = (registers[0] << 16) | registers[1]
        single_turn = angle_raw % self.resolution
        angle_degrees = single_turn * 360.0 / self.resolution
        
        # Try to read circle count separately
        circle_count = 0
        try:
            circle_command = self.build_read_command(0x44, 1)  # Circle count register
            self.ser.reset_input_buffer() 
            self.ser.write(circle_command)
            circle_response = self.ser.read(7)  # ID + FC + BC + 2 bytes + CRC
            circle_result = self.parse_response(circle_response)
            if circle_result:
                circle_registers, _ = circle_result
                if len(circle_registers) >= 1:
                    circle_count = circle_registers[0]
        except:
            circle_count = 0
        
        return {
            'angle_raw': angle_raw,
            'single_turn_value': single_turn,
            'angle_degrees': angle_degrees,
            'speed_raw': 0,
            'speed_rpm': 0.0,
            'circle_count': circle_count,
            'multi_turn_position': circle_count * self.resolution + single_turn,
            'encoder_value': circle_count * self.resolution + single_turn
        }
    
    def read_angle(self):
        """Read only angle value"""
        # Auto-detect resolution if not set
        if self.resolution is None:
            self.resolution = self.detect_resolution()
            
        command = self.build_read_command(0x40, 2)
        self.ser.reset_input_buffer()
        self.ser.write(command)
        response = self.ser.read(9)  # ID + FC + BC + 4 bytes + CRC
        
        result = self.parse_response(response)
        if result:
            registers, _ = result
            angle_raw = (registers[0] << 16) | registers[1]
            single_turn = angle_raw % self.resolution
            return single_turn * 360.0 / self.resolution
        return None
        # ------------------------------------------------------------------
    # QUICK HELPER: read angle for an arbitrary MODBUS ID without
    # opening a second Serial() handle.
    # ------------------------------------------------------------------
    def read_angle_id(self, device_id: int):
        """
        Temporarily talk to *device_id*, call read_angle(), then restore
        the original self.device_id.  Returns None on error just like
        read_angle().
        """
        current_id = self.device_id
        try:
            self.device_id = device_id
            return self.read_angle()
        finally:
            self.device_id = current_id

    def read_all_data_id(self, device_id: int):
        """
        Temporarily talk to *device_id*, call read_all_data(), then restore
        the original self.device_id.  Returns None on error just like
        read_all_data().
        """
        current_id = self.device_id
        try:
            self.device_id = device_id
            return self.read_all_data()
        finally:
            self.device_id = current_id

    def read_speed(self):
        """Read only speed value"""
        command = self.build_read_command(0x42, 2)
        self.ser.reset_input_buffer()
        self.ser.write(command)
        response = self.ser.read(9)  # ID + FC + BC + 4 bytes + CRC
        
        result = self.parse_response(response)
        if result:
            registers, _ = result
            speed_raw = (registers[0] << 16) | registers[1]
            if speed_raw > 0x7FFFFFFF:
                speed_raw = speed_raw - 0x100000000
            return speed_raw / 100.0  # Convert to RPM
        return None
    
    def write_register(self, address, value):
        """
        Write single register (function code 06)
        
        Args:
            address: Register address
            value: Value to write
            
        Returns:
            True if successful, False otherwise
        """
        command = bytearray()
        command.append(self.device_id)
        command.append(0x06)  # Write Single Register
        command.extend(struct.pack('>H', address))
        command.extend(struct.pack('>H', value))
        
        crc = self.calculate_crc16_modbus(command)
        command.extend(struct.pack('<H', crc))
        
        self.ser.reset_input_buffer()
        self.ser.write(command)
        
        # Response should echo the command
        response = self.ser.read(8)
        return response == command
    
    def set_zero_point(self):
        """Set current position as zero point"""
        return self.write_register(0x52, 1)
    
    def set_midpoint(self):
        """Set current position as midpoint"""
        return self.write_register(0x51, 1)
    
    def restart_encoder(self):
        """Restart the encoder"""
        self.write_register(0x54, 1)
        # No response expected after restart
        time.sleep(0.5)  # Wait for encoder to restart

def find_available_port():
    """Find available serial port for encoder communication"""
    # Common USB serial port patterns on Linux
    possible_ports = [
        '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3',
        '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3'
    ]
    
    # Also check for any ttyUSB* ports in /dev
    import glob
    usb_ports = glob.glob('/dev/ttyUSB*')
    acm_ports = glob.glob('/dev/ttyACM*')
    all_ports = possible_ports + usb_ports + acm_ports
    
    # Remove duplicates while preserving order
    seen = set()
    unique_ports = []
    for port in all_ports:
        if port not in seen:
            unique_ports.append(port)
            seen.add(port)
    
    for port in unique_ports:
        if os.path.exists(port):
            try:
                # Try to open the port briefly to check if it's accessible
                test_ser = serial.Serial(port, 1000000, timeout=0.1)
                test_ser.close()
                return port
            except:
                continue
    return None

def diagnose_encoder_ids(port=None, test_ids=None, baudrate=1000000, retries=3):
    """
    Diagnostic script to detect which SF15S03 encoder IDs are online
    
    Args:
        port: Serial port to use (auto-detect if None)
        test_ids: List of IDs to test (default: [1,2,3,4,5,6,7])
        baudrate: Communication speed (default: 1000000)
        retries: Number of retries per ID (default: 3)
    
    Returns:
        Dictionary with diagnostic results
    """
    if test_ids is None:
        test_ids = [1, 2, 3, 4, 5, 6, 7]
    
    if port is None:
        port = find_available_port()
        if port is None:
            print("ERROR: No accessible serial port found!")
            print("Available ports to check manually:")
            for p in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
                if os.path.exists(p):
                    print(f"  {p}")
            return None
    
    print(f"=== SF15S03 Encoder Diagnostic ===")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Testing IDs: {test_ids}")
    print(f"Retries per ID: {retries}")
    print("-" * 50)
    
    online_devices = []
    offline_devices = []
    device_data = {}
    
    try:

        for device_id in test_ids:
            print(f"Testing ID {device_id}...", end=" ")
            encoder = SF15S03Encoder(port=port, device_id=device_id, baudrate=baudrate, timeout=0.1, strict_crc=False)
            
            success_count = 0
            angle_readings = []
            
            # Try multiple times to increase reliability
            for attempt in range(retries):
                try:
                    # Use read_angle_id to test specific device
                    angle = encoder.read_angle()
                    if angle is not None:
                        success_count += 1
                        angle_readings.append(angle)
                    
                    # Small delay between attempts
                    time.sleep(0.01)
                except Exception as e:
                    pass  # Continue to next attempt
            
            # Consider device online if at least half the attempts succeeded
            if success_count >= (retries // 2 + 1):
                online_devices.append(device_id)
                avg_angle = sum(angle_readings) / len(angle_readings) if angle_readings else 0
                device_data[device_id] = {
                    'status': 'ONLINE',
                    'success_rate': f"{success_count}/{retries}",
                    'avg_angle': round(avg_angle, 2),
                    'readings': angle_readings
                }
                print(f"✓ ONLINE ({success_count}/{retries} success, angle: {avg_angle:.2f}°)")
            else:
                offline_devices.append(device_id)
                device_data[device_id] = {
                    'status': 'OFFLINE',
                    'success_rate': f"{success_count}/{retries}",
                    'avg_angle': None,
                    'readings': angle_readings
                }
                print(f"✗ OFFLINE ({success_count}/{retries} success)")
        
        encoder.close()
        
    except Exception as e:
        print(f"\nERROR: Failed to initialize communication: {e}")
        return None
    
    # Print summary
    print("-" * 50)
    print(f"DIAGNOSTIC SUMMARY:")
    print(f"Online devices:  {online_devices} ({len(online_devices)}/{len(test_ids)})")
    print(f"Offline devices: {offline_devices} ({len(offline_devices)}/{len(test_ids)})")
    
    if online_devices:
        print(f"\n✓ Successfully detected {len(online_devices)} encoder(s)")
        print("  Device details:")
        for device_id in online_devices:
            data = device_data[device_id]
            print(f"    ID {device_id}: {data['success_rate']} success, angle: {data['avg_angle']}°")
    
    if offline_devices:
        print(f"\n✗ {len(offline_devices)} device(s) not responding")
        print("  Check connections, power, and device IDs")
    
    return {
        'port': port,
        'online_devices': online_devices,
        'offline_devices': offline_devices,
        'device_data': device_data,
        'total_tested': len(test_ids),
        'total_online': len(online_devices)
    }

if __name__ == "__main__":
    import sys

    enc = SF15S03Encoder(port="/dev/ttyUSB0", baudrate=1000000, timeout=0.01)
    angle_deg = enc.get()
    print(angle_deg)
    enc.close()