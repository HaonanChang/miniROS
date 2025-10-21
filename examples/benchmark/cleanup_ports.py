#!/usr/bin/env python3
"""
Script to clean up ports that might be in use by previous benchmark runs.
"""

import subprocess
import sys
import socket

def kill_process_on_port(port):
    """Kill any process using the specified port."""
    try:
        # Find process using the port
        result = subprocess.run(['lsof', '-ti', f':{port}'], 
                              capture_output=True, text=True)
        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    print(f"Killing process {pid} on port {port}")
                    subprocess.run(['kill', '-9', pid])
                    return True
        return False
    except FileNotFoundError:
        # lsof not available, try netstat
        try:
            result = subprocess.run(['netstat', '-tlnp'], 
                                  capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if f':{port}' in line and 'LISTEN' in line:
                    # Extract PID from netstat output
                    parts = line.split()
                    if len(parts) > 6:
                        pid_program = parts[6]
                        if '/' in pid_program:
                            pid = pid_program.split('/')[0]
                            print(f"Killing process {pid} on port {port}")
                            subprocess.run(['kill', '-9', pid])
                            return True
            return False
        except FileNotFoundError:
            print("Neither lsof nor netstat available for port cleanup")
            return False

def check_port_available(port):
    """Check if a port is available."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('localhost', port))
            return True
    except OSError:
        return False

def main():
    """Main function."""
    ports_to_clean = [5555, 5556, 5557, 5558, 5559]
    
    print("Cleaning up ports...")
    
    for port in ports_to_clean:
        if not check_port_available(port):
            print(f"Port {port} is in use, attempting to clean up...")
            if kill_process_on_port(port):
                print(f"Successfully cleaned up port {port}")
            else:
                print(f"Could not clean up port {port}")
        else:
            print(f"Port {port} is available")
    
    print("Port cleanup completed!")

if __name__ == "__main__":
    main()
