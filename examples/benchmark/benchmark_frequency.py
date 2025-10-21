#!/usr/bin/env python3
"""
Benchmark script to measure the frequency/throughput of client-server communication
using the NetworkQueue with ZMQ.
"""

import numpy as np
import cv2
import sys
import os
import time
import threading
import statistics
import argparse
import socket
from typing import List, Dict, Any
import json
from dataclasses import dataclass

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from mini_ros.utils.net_util import NetUtil
from mini_ros.network.network_queue import NetworkQueueClient, NetworkQueueServer


@dataclass
class BenchmarkResult:
    """Container for benchmark results."""
    total_messages: int
    duration: float
    frequency_hz: float
    avg_latency_ms: float
    min_latency_ms: float
    max_latency_ms: float
    median_latency_ms: float
    std_latency_ms: float
    success_rate: float
    data_type: str
    data_size_bytes: int


class FrequencyBenchmark:
    """Benchmark class for measuring client-server communication frequency."""
    
    def __init__(self, port: int = 5555):
        self.port = port
        self.results = []
        self.server = None
        self.client = None
        self.server_thread = None
        self.benchmark_running = False
        
    def cleanup(self):
        """Clean up server and client resources."""
        if self.client:
            try:
                self.client.socket.close()
            except:
                pass
            self.client = None
            
        if self.server:
            try:
                self.server.socket.close()
            except:
                pass
            self.server = None
            
        if self.server_thread and self.server_thread.is_alive():
            # Give thread time to finish
            self.server_thread.join(timeout=1.0)
        self.server_thread = None
        
        # Give OS time to release the port
        time.sleep(0.1)
    
    def find_available_port(self, start_port: int = 5555, max_attempts: int = 10) -> int:
        """Find an available port starting from start_port."""
        for port in range(start_port, start_port + max_attempts):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                    return port
            except OSError:
                continue
        raise RuntimeError(f"Could not find available port starting from {start_port}")
        
    def create_test_data(self, data_type: str, size: str = "small"):
        """Create test data of different types and sizes."""
        if data_type == "string":
            if size == "small":
                return "Hello, World!"
            elif size == "medium":
                return "A" * 1000
            else:  # large
                return "A" * 10000
                
        elif data_type == "numpy":
            if size == "small":
                return np.random.rand(10, 10)
            elif size == "medium":
                return np.random.rand(100, 100)
            else:  # large
                return np.random.rand(500, 500)
                
        elif data_type == "image":
            if size == "small":
                return np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)
            elif size == "medium":
                return np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
            else:  # large
                return np.random.randint(0, 255, (512, 512, 3), dtype=np.uint8)
                
        elif data_type == "dict":
            if size == "small":
                return {"id": 1, "value": 42.0}
            elif size == "medium":
                return {f"key_{i}": f"value_{i}" for i in range(100)}
            else:  # large
                return {f"key_{i}": f"value_{i}" for i in range(1000)}
    
    def start_server(self, data_type: str = 'auto'):
        """Start the server in a separate thread."""
        # Clean up any existing server first
        self.cleanup()
        
        # Try to find an available port
        try:
            self.port = self.find_available_port(self.port)
        except RuntimeError as e:
            print(f"Warning: {e}, using port {self.port}")
        
        self.server = NetworkQueueServer("benchmark_server", self.port, data_type=data_type)
        self.server_thread = threading.Thread(target=self.server.start_in_thread, daemon=True)
        self.server_thread.start()
        time.sleep(0.5)  # Give server time to start
        
    def run_benchmark(self, 
                     data_type: str = "string", 
                     data_size: str = "small",
                     num_messages: int = 1000,
                     timeout: float = 5.0) -> BenchmarkResult:
        """Run a single benchmark test."""
        print(f"Running benchmark: {data_type} ({data_size}), {num_messages} messages")
        
        # Create test data
        test_data = self.create_test_data(data_type, data_size)
        data_size_bytes = len(str(test_data).encode('utf-8')) if isinstance(test_data, str) else test_data.nbytes if hasattr(test_data, 'nbytes') else len(str(test_data))
        
        # Start server
        self.start_server(data_type)
        
        # Create client
        self.client = NetworkQueueClient("benchmark_client", self.port, data_type=data_type)
        
        # Benchmark variables
        latencies = []
        successful_sends = 0
        start_time = time.time()
        
        print(f"Starting benchmark at {time.strftime('%H:%M:%S')}")
        
        for i in range(num_messages):
            if not self.benchmark_running:
                break
                
            # Measure latency
            send_start = time.time()
            success = self.client.put(test_data, code="benchmark")
            send_end = time.time()
            
            if success:
                successful_sends += 1
                latency_ms = (send_end - send_start) * 1000
                latencies.append(latency_ms)
                
                # Progress indicator
                if (i + 1) % 100 == 0:
                    print(f"Progress: {i + 1}/{num_messages} messages sent")
            else:
                print(f"Failed to send message {i + 1}")
                
        end_time = time.time()
        duration = end_time - start_time
        
        # Calculate statistics
        frequency_hz = successful_sends / duration if duration > 0 else 0
        success_rate = (successful_sends / num_messages) * 100 if num_messages > 0 else 0
        
        if latencies:
            avg_latency = statistics.mean(latencies)
            min_latency = min(latencies)
            max_latency = max(latencies)
            median_latency = statistics.median(latencies)
            std_latency = statistics.stdev(latencies) if len(latencies) > 1 else 0
        else:
            avg_latency = min_latency = max_latency = median_latency = std_latency = 0
        
        result = BenchmarkResult(
            total_messages=num_messages,
            duration=duration,
            frequency_hz=frequency_hz,
            avg_latency_ms=avg_latency,
            min_latency_ms=min_latency,
            max_latency_ms=max_latency,
            median_latency_ms=median_latency,
            std_latency_ms=std_latency,
            success_rate=success_rate,
            data_type=data_type,
            data_size_bytes=data_size_bytes
        )
        
        self.results.append(result)
        return result
    
    def run_comprehensive_benchmark(self, 
                                  data_types: List[str] = None,
                                  data_sizes: List[str] = None,
                                  num_messages: int = 1000):
        """Run comprehensive benchmark across different data types and sizes."""
        if data_types is None:
            data_types = ["string", "numpy", "image", "dict"]
        if data_sizes is None:
            data_sizes = ["small", "medium", "large"]
            
        print("=" * 80)
        print("COMPREHENSIVE FREQUENCY BENCHMARK")
        print("=" * 80)
        
        self.benchmark_running = True
        
        for data_type in data_types:
            for data_size in data_sizes:
                try:
                    result = self.run_benchmark(data_type, data_size, num_messages)
                    self.print_result(result)
                    time.sleep(1)  # Brief pause between tests
                except Exception as e:
                    print(f"Error in benchmark {data_type} ({data_size}): {e}")
                finally:
                    # Clean up
                    self.cleanup()
                    time.sleep(0.5)
        
        self.benchmark_running = False
        self.print_summary()
    
    def print_result(self, result: BenchmarkResult):
        """Print a single benchmark result."""
        print(f"\n{'-' * 60}")
        print(f"Data Type: {result.data_type} ({result.data_size_bytes} bytes)")
        print(f"Messages: {result.total_messages}")
        print(f"Duration: {result.duration:.2f}s")
        print(f"Frequency: {result.frequency_hz:.2f} Hz")
        print(f"Success Rate: {result.success_rate:.1f}%")
        print(f"Latency (ms):")
        print(f"  Average: {result.avg_latency_ms:.2f}")
        print(f"  Median:  {result.median_latency_ms:.2f}")
        print(f"  Min:     {result.min_latency_ms:.2f}")
        print(f"  Max:     {result.max_latency_ms:.2f}")
        print(f"  Std Dev: {result.std_latency_ms:.2f}")
    
    def print_summary(self):
        """Print summary of all benchmark results."""
        if not self.results:
            print("No benchmark results to summarize.")
            return
            
        print("\n" + "=" * 80)
        print("BENCHMARK SUMMARY")
        print("=" * 80)
        
        # Group results by data type
        by_type = {}
        for result in self.results:
            if result.data_type not in by_type:
                by_type[result.data_type] = []
            by_type[result.data_type].append(result)
        
        for data_type, results in by_type.items():
            print(f"\n{data_type.upper()} RESULTS:")
            print("-" * 40)
            
            # Find best frequency for each size
            by_size = {}
            for result in results:
                size = "unknown"
                if result.data_size_bytes < 1000:
                    size = "small"
                elif result.data_size_bytes < 10000:
                    size = "medium"
                else:
                    size = "large"
                
                if size not in by_size or result.frequency_hz > by_size[size].frequency_hz:
                    by_size[size] = result
            
            for size, result in by_size.items():
                print(f"  {size}: {result.frequency_hz:.2f} Hz "
                      f"(avg latency: {result.avg_latency_ms:.2f}ms)")
        
        # Overall best performance
        best_result = max(self.results, key=lambda r: r.frequency_hz)
        print(f"\nBEST OVERALL PERFORMANCE:")
        print(f"  {best_result.data_type} ({best_result.data_size_bytes} bytes): "
              f"{best_result.frequency_hz:.2f} Hz")
    
    def save_results(self, filename: str = "benchmark_results.json"):
        """Save benchmark results to JSON file."""
        results_dict = []
        for result in self.results:
            results_dict.append({
                "total_messages": result.total_messages,
                "duration": result.duration,
                "frequency_hz": result.frequency_hz,
                "avg_latency_ms": result.avg_latency_ms,
                "min_latency_ms": result.min_latency_ms,
                "max_latency_ms": result.max_latency_ms,
                "median_latency_ms": result.median_latency_ms,
                "std_latency_ms": result.std_latency_ms,
                "success_rate": result.success_rate,
                "data_type": result.data_type,
                "data_size_bytes": result.data_size_bytes
            })
        
        with open(filename, 'w') as f:
            json.dump(results_dict, f, indent=2)
        print(f"Results saved to {filename}")


def main():
    """Main function with command line interface."""
    parser = argparse.ArgumentParser(description="Benchmark client-server communication frequency")
    parser.add_argument("--port", type=int, default=5555, help="Port for communication")
    parser.add_argument("--messages", type=int, default=1000, help="Number of messages to send")
    parser.add_argument("--data-types", nargs="+", default=["string", "numpy", "image", "dict"],
                       help="Data types to test")
    parser.add_argument("--data-sizes", nargs="+", default=["small", "medium", "large"],
                       help="Data sizes to test")
    parser.add_argument("--output", type=str, default="benchmark_results.json",
                       help="Output file for results")
    parser.add_argument("--quick", action="store_true", help="Run quick test (100 messages)")
    
    args = parser.parse_args()
    
    if args.quick:
        args.messages = 100
        args.data_types = ["string", "numpy"]
        args.data_sizes = ["small", "medium"]
    
    benchmark = FrequencyBenchmark(port=args.port)
    
    try:
        benchmark.run_comprehensive_benchmark(
            data_types=args.data_types,
            data_sizes=args.data_sizes,
            num_messages=args.messages
        )
        benchmark.save_results(args.output)
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by user")
        benchmark.benchmark_running = False
    except Exception as e:
        print(f"Benchmark failed: {e}")
    finally:
        # Ensure cleanup
        benchmark.cleanup()


if __name__ == "__main__":
    main()
