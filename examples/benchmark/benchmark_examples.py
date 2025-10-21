#!/usr/bin/env python3
"""
Simple examples showing how to use the frequency benchmark script.
"""

import subprocess
import sys
import os

def run_quick_benchmark():
    """Run a quick benchmark test."""
    print("Running quick benchmark (100 messages)...")
    cmd = [sys.executable, "benchmark_frequency.py", "--quick"]
    subprocess.run(cmd, cwd=os.path.dirname(__file__))

def run_full_benchmark():
    """Run a full benchmark test."""
    print("Running full benchmark (1000 messages)...")
    cmd = [sys.executable, "benchmark_frequency.py", "--messages", "1000"]
    subprocess.run(cmd, cwd=os.path.dirname(__file__))

def run_custom_benchmark():
    """Run a custom benchmark with specific parameters."""
    print("Running custom benchmark...")
    cmd = [
        sys.executable, "benchmark_frequency.py",
        "--messages", "500",
        "--data-types", "string", "numpy",
        "--data-sizes", "small", "medium",
        "--output", "custom_results.json"
    ]
    subprocess.run(cmd, cwd=os.path.dirname(__file__))

def run_high_frequency_test():
    """Run a high-frequency test with many small messages."""
    print("Running high-frequency test...")
    cmd = [
        sys.executable, "benchmark_frequency.py",
        "--messages", "5000",
        "--data-types", "string",
        "--data-sizes", "small",
        "--output", "high_freq_results.json"
    ]
    subprocess.run(cmd, cwd=os.path.dirname(__file__))

if __name__ == "__main__":
    print("Frequency Benchmark Examples")
    print("=" * 40)
    print("1. Quick benchmark (100 messages)")
    print("2. Full benchmark (1000 messages)")
    print("3. Custom benchmark (500 messages, specific types)")
    print("4. High-frequency test (5000 small messages)")
    
    choice = input("\nSelect an option (1-4): ").strip()
    
    if choice == "1":
        run_quick_benchmark()
    elif choice == "2":
        run_full_benchmark()
    elif choice == "3":
        run_custom_benchmark()
    elif choice == "4":
        run_high_frequency_test()
    else:
        print("Invalid choice. Running quick benchmark...")
        run_quick_benchmark()
