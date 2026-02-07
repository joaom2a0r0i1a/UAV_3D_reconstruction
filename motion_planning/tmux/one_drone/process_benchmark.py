import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import io

# --- CONFIGURATION ---
FILE_PATH = 'aeplanner_benchmark_2.csv' 

def process_data():
    # 1. Load Data
    try:
        df = pd.read_csv(FILE_PATH)
    except FileNotFoundError:
        print(f"Error: Could not find {FILE_PATH}.")
        return

    # --- DATA ADAPTATION (PER YOUR REQUEST) ---
    # We rename the existing 'Total_GPU_Time_ms' to 'Total_GPU_Abs_Time_ms'
    # And we duplicate it to create 'Total_GPU_Marg_Time_ms' for now.
    
    if 'Total_GPU_Abs_Time_ms' not in df.columns:
        print("Adapting dataset structure...")
        df.rename(columns={'Total_GPU_Time_ms': 'Total_GPU_Abs_Time_ms'}, inplace=True)
        # Duplicate for testing purposes as requested
        df['Total_GPU_Marg_Time_ms'] = df['Total_GPU_Abs_Time_ms'] 

    # 2. Feature Engineering (Calculate "Per Node" metrics)
    df['CPU_Per_Node_ms']       = df['Total_CPU_Time_ms'] / df['Total_Nodes']
    df['GPU_Abs_Per_Node_ms']   = df['Total_GPU_Abs_Time_ms'] / df['Total_Nodes']
    df['GPU_Marg_Per_Node_ms']  = df['Total_GPU_Marg_Time_ms'] / df['Total_Nodes']
    
    # Calculate Speedups
    df['Speedup_Abs']  = df['Total_CPU_Time_ms'] / (df['Total_GPU_Abs_Time_ms'] + 1e-9)
    df['Speedup_Marg'] = df['Total_CPU_Time_ms'] / (df['Total_GPU_Marg_Time_ms'] + 1e-9)

    # 3. Compute Statistics
    stats = {
        'total_samples': len(df),
        'cpu_mean': df['CPU_Per_Node_ms'].mean(), 'cpu_std': df['CPU_Per_Node_ms'].std(),
        'gpu_abs_mean': df['GPU_Abs_Per_Node_ms'].mean(), 'gpu_abs_std': df['GPU_Abs_Per_Node_ms'].std(),
        'gpu_marg_mean': df['GPU_Marg_Per_Node_ms'].mean(), 'gpu_marg_std': df['GPU_Marg_Per_Node_ms'].std(),
        'speedup_abs_mean': df['Speedup_Abs'].mean(),
        'speedup_marg_mean': df['Speedup_Marg'].mean(),
    }

    # Print Report
    print("="*60)
    print(f"BENCHMARK RESULTS (N={stats['total_samples']} batches)")
    print("="*60)
    print(f"CPU Baseline     : {stats['cpu_mean']:.3f} ± {stats['cpu_std']:.3f} ms/node")
    print(f"GPU Absolute     : {stats['gpu_abs_mean']:.3f} ± {stats['gpu_abs_std']:.3f} ms/node")
    print(f"GPU Marginal     : {stats['gpu_marg_mean']:.3f} ± {stats['gpu_marg_std']:.3f} ms/node")
    print("-" * 60)
    print(f"Speedup (Abs)    : {stats['speedup_abs_mean']:.2f} x")
    print(f"Speedup (Marg)   : {stats['speedup_marg_mean']:.2f} x")
    print("="*60)

    # --- 4. VISUALIZATION ---
    sns.set_style("whitegrid")
    plt.rcParams.update({'font.size': 12, 'font.family': 'sans-serif'})

    # Melt Data for 3-Way Box Plot
    df_melted = df.melt(
        value_vars=['CPU_Per_Node_ms', 'GPU_Abs_Per_Node_ms', 'GPU_Marg_Per_Node_ms'], 
        var_name='Processing Unit', 
        value_name='Time per Node (ms)'
    )
    
    df_melted['Processing Unit'] = df_melted['Processing Unit'].replace({
        'CPU_Per_Node_ms': 'CPU\n(Baseline)', 
        'GPU_Abs_Per_Node_ms': 'GPU\n(Absolute)',
        'GPU_Marg_Per_Node_ms': 'GPU\n(Marginal)'
    })

    # FIGURE A: 3-Way Box Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    palette_colors = ["#d62728", "#1f77b4", "#2ca02c"] # Red, Blue, Green

    sns.boxplot(x='Processing Unit', y='Time per Node (ms)', data=df_melted, 
                palette=palette_colors, width=0.6, showfliers=False, ax=ax, linewidth=1.5)
    sns.stripplot(x='Processing Unit', y='Time per Node (ms)', data=df_melted, 
                  color=".15", alpha=0.3, size=3, jitter=True, ax=ax)

    ax.set_title('Computation Time Comparison', fontweight='bold', pad=12)
    ax.set_xlabel('') 
    ax.set_ylabel('Time per Candidate (ms)')
    ax.set_ylim(bottom=0) # Linear scale starting at 0
    ax.yaxis.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.savefig('benchmark_boxplot_3way.png', dpi=300)
    print("Saved 'benchmark_boxplot_3way.png'")

    # FIGURE B: Speedup Histogram
    fig2, ax2 = plt.subplots(figsize=(8, 5))
    
    # Overlap Histograms
    sns.histplot(df['Speedup_Abs'], color="#1f77b4", label='GPU Absolute', kde=True, element="step", alpha=0.3, ax=ax2)
    sns.histplot(df['Speedup_Marg'], color="#2ca02c", label='GPU Marginal', kde=True, element="step", alpha=0.3, ax=ax2)
    
    ax2.axvline(stats['speedup_abs_mean'], color='#1f77b4', linestyle='--', linewidth=2, label=f"Mean Abs: {stats['speedup_abs_mean']:.1f}x")
    ax2.axvline(stats['speedup_marg_mean'], color='#2ca02c', linestyle='--', linewidth=2, label=f"Mean Marg: {stats['speedup_marg_mean']:.1f}x")
    
    ax2.set_title('Distribution of Speedup Factors', fontweight='bold', pad=12)
    ax2.set_xlabel('Speedup Factor ($t_{cpu} / t_{gpu}$)')
    ax2.set_ylabel('Frequency')
    ax2.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('benchmark_speedup_3way.png', dpi=300)
    print("Saved 'benchmark_speedup_3way.png'")

    plt.show()

if __name__ == "__main__":
    process_data()