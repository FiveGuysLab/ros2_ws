#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def analyze_latency(csv_path):
    # Read the CSV file
    df = pd.read_csv(csv_path)
    
    # Calculate statistics
    latency_ms = df['latency_sec'] * 1000 + df['latency_nanosec'] / 1_000_000  # Convert to milliseconds
    
    stats = {
        'mean': latency_ms.mean(),
        'median': latency_ms.median(),
        'std': latency_ms.std(),
        'min': latency_ms.min(),
        'max': latency_ms.max(),
        'p95': latency_ms.quantile(0.95),
        'p99': latency_ms.quantile(0.99)
    }
    
    # Print statistics
    print("\nLatency Statistics (in milliseconds):")
    print(f"Mean latency: {stats['mean']:.3f} ms")
    print(f"Median latency: {stats['median']:.3f} ms")
    print(f"Standard deviation: {stats['std']:.3f} ms")
    print(f"Min latency: {stats['min']:.3f} ms")
    print(f"Max latency: {stats['max']:.3f} ms")
    print(f"95th percentile: {stats['p95']:.3f} ms")
    print(f"99th percentile: {stats['p99']:.3f} ms")
    
    # Create plots
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Latency over time
    plt.subplot(2, 2, 1)
    plt.plot(range(len(latency_ms)), latency_ms, 'b-', alpha=0.5)
    plt.title('Latency over Time')
    plt.xlabel('Message Number')
    plt.ylabel('Latency (ms)')
    plt.grid(True)
    
    # Plot 2: Histogram
    plt.subplot(2, 2, 2)
    plt.hist(latency_ms, bins=50, edgecolor='black')
    plt.title('Latency Distribution')
    plt.xlabel('Latency (ms)')
    plt.ylabel('Count')
    plt.grid(True)
    
    # Plot 3: Box plot
    plt.subplot(2, 2, 3)
    plt.boxplot(latency_ms)
    plt.title('Latency Box Plot')
    plt.ylabel('Latency (ms)')
    plt.grid(True)
    
    # Plot 4: Rolling average
    window_size = 50
    plt.subplot(2, 2, 4)
    plt.plot(range(len(latency_ms)), latency_ms.rolling(window=window_size).mean(), 'r-')
    plt.title(f'Rolling Average (window={window_size})')
    plt.xlabel('Message Number')
    plt.ylabel('Latency (ms)')
    plt.grid(True)
    
    plt.tight_layout()
    
    # Save plots
    plt.savefig('latency_analysis.png')
    print("\nPlots saved as 'latency_analysis.png'")
    
    # Save detailed statistics to file
    with open('latency_stats.txt', 'w') as f:
        f.write("Latency Statistics (in milliseconds):\n")
        for key, value in stats.items():
            f.write(f"{key}: {value:.3f} ms\n")
        
        # Add more detailed analysis
        f.write("\nPercentile Analysis:\n")
        percentiles = [1, 5, 10, 25, 50, 75, 90, 95, 99, 99.9]
        for p in percentiles:
            value = latency_ms.quantile(p/100)
            f.write(f"{p}th percentile: {value:.3f} ms\n")
            
        # Add stability analysis
        f.write("\nStability Analysis:\n")
        chunk_size = len(latency_ms) // 10  # Divide data into 10 chunks
        for i in range(10):
            chunk = latency_ms[i*chunk_size:(i+1)*chunk_size]
            f.write(f"Chunk {i+1} mean: {chunk.mean():.3f} ms, std: {chunk.std():.3f} ms\n")

if __name__ == "__main__":
    csv_path = "subscriber_timestamps.csv"
    if not Path(csv_path).exists():
        print(f"Error: Could not find {csv_path}")
        exit(1)
    
    analyze_latency(csv_path) 