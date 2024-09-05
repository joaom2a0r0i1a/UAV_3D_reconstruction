import pandas as pd
import matplotlib.pyplot as plt
import csv
import os
import math

def plot_csv(file_path):
    df = pd.read_csv(file_path)

    if df.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")
    
    df.columns = ['NumberNodes', 'GainComputationTime']

    df = df.sort_values(by='NumberNodes')

    num_nodes = df['NumberNodes']
    gain_time = df['GainComputationTime']
    cumulative_avg = gain_time.expanding().mean()

    plt.figure(figsize=(10, 6))
    plt.plot(num_nodes, gain_time, linestyle='-', color='b', label='Gain Computation Time')
    plt.plot(num_nodes, cumulative_avg, linestyle='-', color='r', label='Cumulative Average Gain Computation Time')

    plt.xlabel('Number of Nodes')
    plt.ylabel('Gain Computation Time')
    plt.title('Gain Computation Time')
    plt.legend()
    plt.grid(True)

    plt.show()

def plot_both_csv(file_path1, file_path2):
    df = pd.read_csv(file_path1)
    df2 = pd.read_csv(file_path2)

    if df.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")

    df.columns = ['NumberNodes', 'GainComputationTime']
    df = df.sort_values(by='NumberNodes')

    num_nodes = df['NumberNodes']
    gain_time = df['GainComputationTime']
    cumulative_avg = gain_time.expanding().mean()

    if df2.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")

    df2.columns = ['NumberNodes', 'GainComputationTime']
    df2 = df2.sort_values(by='NumberNodes')

    num_nodes2 = df2['NumberNodes']
    gain_time2 = df2['GainComputationTime']
    cumulative_avg2 = gain_time2.expanding().mean()

    # Create the plot
    plt.figure(figsize=(10, 6))
    #plt.plot(num_nodes, gain_time, linestyle='-', color='b', label='Gain Computation Time')
    #plt.plot(num_nodes2, gain_time2, linestyle='-', color='y', label='Sparse Gain Computation Time')

    plt.plot(num_nodes, cumulative_avg, linestyle='-', color='r', label='Cumulative Average Gain Computation Time')
    plt.plot(num_nodes2, cumulative_avg2, linestyle='-', color='g', label='Sparse Cumulative Average Gain Computation Time')

    plt.xlabel('Number of Nodes')
    plt.ylabel('Gain Computation Time')
    plt.title('Gain Computation Time')
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()

def compute_average(csv_file):
    with open(csv_file, mode='r') as file:
        reader = csv.reader(file)
        next(reader)
        total = 0
        count = 0
        for row in reader:
            total += float(row[1])
            count += 1 # get last row to get count number
        average = total / count
        return average

def compute_overall_stats(directory):
    averages = []
    
    for filename in os.listdir(directory):
        if filename.endswith('.csv'):
            csv_file = os.path.join(directory, filename)
            avg = compute_average(csv_file)
            averages.append(avg)
    
    if len(averages) == 0:
        raise ValueError("No CSV files found in the directory.")

    overall_average = sum(averages) / len(averages)
    variance = sum((x - overall_average) ** 2 for x in averages) / len(averages)
    standard_deviation = math.sqrt(variance)
    
    return overall_average, standard_deviation

def compute_and_plot_bar_graph(directory):
    total_overall_avg = [] 
    total_std_dev = [] 
    directory_name = []
    
    for filename in os.listdir(directory):
        if filename.startswith('sparse'):
            directory_aux = os.path.join(directory, filename)
            directory_name.append(filename)
            overall_avg, std_dev = compute_overall_stats(directory_aux)
            total_overall_avg.append(overall_avg)
            total_std_dev.append(total_std_dev)

    print(f"New Time is: {total_overall_avg[0]/total_overall_avg[1]:.2f} times faster.")

    plt.figure(figsize=(8, 6))

    plt.bar(directory_name, total_overall_avg, yerr=std_dev, capsize=5, color='r', alpha=0.9)
    
    plt.xlabel('Raycasting Methods')
    plt.ylabel('Computation Time (s)')
    plt.title('Mean and Standard Deviation of Gain Computation Time')
    #plt.legend()
    plt.grid(False)
    plt.tight_layout()
    plt.show()


# Example usage:
directory = '/home/joaomendes/motion_workspace/src/data'
compute_and_plot_bar_graph(directory)
#overall_avg, std_dev = compute_overall_stats(directory)
#print(f"Overall Average: {overall_avg}")
#print(f"Standard Deviation: {std_dev}")

#csv_file_path = '/home/joaomendes/motion_workspace/src/data/sparse_time_optimized/computation_time_gain_20240904_151932.csv'
#csv_file_path2 = '/home/joaomendes/motion_workspace/src/data/sparse_time_optimized_array/computation_time_gain_20240904_134548.csv'

# Plot the data
#plot_csv(csv_file_path)
#plot_both_csv(csv_file_path, csv_file_path2)
