import numpy as np
import cv2
import pickle
import base64
from typing import Union, Any, Dict, Tuple, List
import io


class NetUtil:
    """
    Utility functions for network operations, including encoding/decoding
    numpy arrays and images for ZMQ communication.
    """

    @classmethod
    def encode(cls, data: Any, data_type: str) -> bytes:
        """
        Encode data to bytes for ZMQ transmission.
        """
        if data_type == 'ndarray' or data_type == 'numpy':
            return cls.encode_ndarray(data)
        elif data_type == 'image':
            return cls.encode_image(data)
        elif data_type == 'string' or data_type == 'str':
            return cls.encode_string(data)
        elif data_type == 'dict':
            return cls.encode_dict(data)
        else:
            raise ValueError(f"Invalid data_type: {data_type}. Must be 'ndarray', 'numpy', 'image', 'string'")
    
    @classmethod
    def encode_dict(cls, data: Dict) -> bytes:
        """
        Encode a dictionary to bytes for ZMQ transmission.
        """
        if not isinstance(data, dict):
            raise ValueError("Input must be a dictionary")
        return pickle.dumps(data)

    @classmethod
    def encode_ndarray(cls, array: np.ndarray) -> bytes:
        """
        Encode a numpy array to bytes for ZMQ transmission.
        
        Args:
            array: The numpy array to encode
            
        Returns:
            bytes: Encoded array data
            
        Raises:
            ValueError: If input is not a numpy array
        """
        if not isinstance(array, np.ndarray):
            raise ValueError("Input must be a numpy array")
        
        # Create a dictionary with array data and metadata
        array_data = {
            'data': array.tobytes(),
            'dtype': str(array.dtype),
            'shape': array.shape
        }
        
        # Serialize using pickle
        return pickle.dumps(array_data)
    
    @classmethod
    def encode_image(cls, image: np.ndarray, format: str = 'jpg', quality: int = 95) -> bytes:
        """
        Encode an image (numpy array) to bytes for ZMQ transmission.
        
        Args:
            image: The image as a numpy array (BGR format for OpenCV)
            format: Image format ('jpg', 'png', 'webp')
            quality: JPEG quality (1-100, only used for JPEG)
            
        Returns:
            bytes: Encoded image data
            
        Raises:
            ValueError: If input is not a numpy array or invalid format
        """
        if not isinstance(image, np.ndarray):
            raise ValueError("Input must be a numpy array")
        
        if len(image.shape) not in [2, 3]:
            raise ValueError("Image must be 2D (grayscale) or 3D (color)")
        
        # Validate format
        valid_formats = ['jpg', 'jpeg', 'png', 'webp']
        if format.lower() not in valid_formats:
            raise ValueError(f"Format must be one of {valid_formats}")
        
        # Validate quality for JPEG
        if format.lower() in ['jpg', 'jpeg']:
            quality = max(1, min(100, quality))
        
        # Encode image
        if format.lower() in ['jpg', 'jpeg']:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        elif format.lower() == 'png':
            encode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), 9]
        elif format.lower() == 'webp':
            encode_param = [int(cv2.IMWRITE_WEBP_QUALITY), quality]
        else:
            encode_param = []
        
        success, encoded_img = cv2.imencode(f'.{format.lower()}', image, encode_param)
        
        if not success:
            raise ValueError(f"Failed to encode image as {format}")
        
        return encoded_img.tobytes()
    
    @classmethod
    def encode_string(cls, string: str) -> bytes:
        """
        Encode a string to bytes for ZMQ transmission.
        """
        return string.encode('utf-8')
    
    @classmethod
    def decode(cls, data: bytes, data_type: str = 'auto') -> Any:
        """
        Decode bytes back to original format.
        
        Args:
            data: The encoded data bytes
            data_type: Type of data ('auto', 'ndarray', 'image', 'string', 'pickle')
            
        Returns:
            Decoded data (numpy array, image, string, or any pickled object)
            
        Raises:
            ValueError: If data_type is invalid or decoding fails
        """
        if not isinstance(data, bytes):
            raise ValueError("Input must be bytes")
        
        if data_type == 'auto':
            # Try to detect the data type
            try:
                # First try to decode as pickled numpy array
                array_data = pickle.loads(data)
                if isinstance(array_data, dict) and 'data' in array_data and 'dtype' in array_data and 'shape' in array_data:
                    return cls._decode_ndarray_from_dict(array_data)
            except:
                pass
            
            try:
                # Try to decode as image
                return cls._decode_image(data)
            except:
                pass
            
            try:
                # Try to decode as UTF-8 string
                decoded_str = data.decode('utf-8')
                # Check if it looks like a valid string (not just random bytes)
                if decoded_str.isprintable() or '\n' in decoded_str or '\t' in decoded_str:
                    return decoded_str
            except:
                pass
            
            # Fallback to pickle
            return pickle.loads(data)
        
        elif data_type == 'ndarray' or data_type == 'numpy':
            array_data = pickle.loads(data)
            if isinstance(array_data, dict) and 'data' in array_data and 'dtype' in array_data and 'shape' in array_data:
                return cls._decode_ndarray_from_dict(array_data)
            else:
                raise ValueError("Data is not a valid encoded numpy array")
        
        elif data_type == 'image':
            return cls._decode_image(data)
        
        elif data_type == 'string' or data_type == 'str':
            return cls._decode_string(data)
        
        elif data_type == 'pickle':
            return pickle.loads(data)
        
        elif data_type == 'dict':
            return pickle.loads(data)
        
        else:
            raise ValueError(f"Invalid data_type: {data_type}. Must be 'auto', 'ndarray', 'numpy', 'image', 'string', or 'pickle'")
    
    @classmethod
    def _decode_ndarray_from_dict(cls, array_data: Dict) -> np.ndarray:
        """Helper method to decode numpy array from dictionary."""
        return np.frombuffer(
            array_data['data'], 
            dtype=array_data['dtype']
        ).reshape(array_data['shape'])
    
    @classmethod
    def _decode_string(cls, data: bytes) -> str:
        """Helper method to decode string from bytes."""
        try:
            return data.decode('utf-8')
        except UnicodeDecodeError as e:
            raise ValueError(f"Failed to decode string data: {e}")
    
    @classmethod
    def _decode_image(cls, data: bytes) -> np.ndarray:
        """Helper method to decode image from bytes."""
        nparr = np.frombuffer(data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode image data")
        return image