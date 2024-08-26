import pandas as pd
import matplotlib.pyplot as plt

def plot_csv(file_path):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Check if the CSV file has at least two columns
    if df.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")

    # Ensure correct column names or use default names
    df.columns = ['NumberNodes', 'GainComputationTime']  # Explicitly set column names

    # Sort DataFrame by 'NumberNodes' to ensure correct order
    df = df.sort_values(by='NumberNodes')

    # Extract the columns for plotting
    num_nodes = df['NumberNodes']
    gain_time = df['GainComputationTime']

    # Compute cumulative average
    cumulative_avg = gain_time.expanding().mean()

    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(num_nodes, gain_time, linestyle='-', color='b', label='Gain Computation Time')
    plt.plot(num_nodes, cumulative_avg, linestyle='-', color='r', label='Cumulative Average Gain Computation Time')

    # Labeling the plot
    plt.xlabel('Number of Nodes')
    plt.ylabel('Gain Computation Time')
    plt.title('Gain Computation Time')
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()

def plot_both_csv(file_path1, file_path2):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path1)
    df2 = pd.read_csv(file_path2)

    # Check if the CSV file has at least two columns
    if df.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")

    # Ensure correct column names or use default names
    df.columns = ['NumberNodes', 'GainComputationTime']  # Explicitly set column names

    # Sort DataFrame by 'NumberNodes' to ensure correct order
    df = df.sort_values(by='NumberNodes')

    # Extract the columns for plotting
    num_nodes = df['NumberNodes']
    gain_time = df['GainComputationTime']

    # Compute cumulative average
    cumulative_avg = gain_time.expanding().mean()

    if df2.shape[1] < 2:
        raise ValueError("CSV file must contain at least two columns")

    # Ensure correct column names or use default names
    df2.columns = ['NumberNodes', 'GainComputationTime']  # Explicitly set column names

    # Sort DataFrame by 'NumberNodes' to ensure correct order
    df2 = df2.sort_values(by='NumberNodes')

    # Extract the columns for plotting
    num_nodes2 = df2['NumberNodes']
    gain_time2 = df2['GainComputationTime']

    # Compute cumulative average
    cumulative_avg2 = gain_time2.expanding().mean()

    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(num_nodes, gain_time, linestyle='-', color='b', label='Gain Computation Time')
    plt.plot(num_nodes, cumulative_avg, linestyle='-', color='r', label='Cumulative Average Gain Computation Time')

    plt.plot(num_nodes2, gain_time2, linestyle='-', color='y', label='Sparse Gain Computation Time')
    plt.plot(num_nodes2, cumulative_avg2, linestyle='-', color='g', label='Sparse Cumulative Average Gain Computation Time')

    # Labeling the plot
    plt.xlabel('Number of Nodes')
    plt.ylabel('Gain Computation Time')
    plt.title('Gain Computation Time')
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()

# Replace with your CSV file path
csv_file_path = '/home/joaomendes/motion_workspace/src/data/computation_time_gain_raycast.csv'
csv_file_path2 = '/home/joaomendes/motion_workspace/src/data/computation_time_gain_ours.csv'

# Plot the data
#plot_csv(csv_file_path)
plot_both_csv(csv_file_path, csv_file_path2)
