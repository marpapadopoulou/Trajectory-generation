import pickle
import numpy as np
import matplotlib.pyplot as plt

def plot_velocity_trends(ee_vel_log):
    # Flatten the list of arrays into a single array for each dimension of velocity
    all_velocities = np.concatenate(ee_vel_log, axis=0)
    
    time_steps = np.arange(len(all_velocities))

    # Plot each component of the velocity
    plt.figure(figsize=(12, 8))
    for i in range(all_velocities.shape[1]):
        plt.plot(time_steps, all_velocities[:, i], label=f'Velocity Component {i+1}')
    
    plt.title('Trends in End-Effector Velocities with Non-Stationary Noise')
    plt.xlabel('Time Steps')
    plt.ylabel('Velocity')
    plt.legend()
    plt.show()
    
def plot_individual_samples(ee_vel_log):
    num_samples = len(ee_vel_log)
    fig, axs = plt.subplots(num_samples, 3, figsize=(18, 6 * num_samples), sharex=True)
    
    for j, sample in enumerate(ee_vel_log):
        time_steps = np.arange(sample.shape[0])
        for i in range(sample.shape[1]):
            axs[j, i].plot(time_steps, sample[:, i], label=f'Sample {j+1} Velocity Component {i+1}')
            axs[j, i].set_ylabel(f'Sample {j+1} Velocity Component {i+1}')
            axs[j, i].legend()
            axs[j, i].grid(True)

    for i in range(3):
        axs[-1, i].set_xlabel('Time Steps')

    plt.suptitle('Trends in End-Effector Velocities for Each Sample')
    plt.show()

def plot_single_sample(sample, sample_index):
    time_steps = np.arange(sample.shape[0])

    fig, axs = plt.subplots(3, 1, figsize=(12, 18), sharex=True)

    for i in range(sample.shape[1]):
        axs[i].plot(time_steps, sample[:, i], label=f'Velocity Component {i+1}')
        axs[i].set_ylabel(f'Velocity Component {i+1}')
        axs[i].legend()
        axs[i].grid(True)

    axs[2].set_xlabel('Time Steps')
    plt.suptitle(f'Trends in End-Effector Velocities for Sample {sample_index}')
    plt.show()

if __name__ == "__main__":
    # Load the data from the pickle file
    with open('Traj20k-dataset-std0.01.json', 'rb') as fp:
        data = pickle.load(fp)

    # Extract the ee_vel_log from the loaded data
    ee_vel_log = data['ee_vel_log']

    # Plot the velocity trends
    #plot_velocity_trends(ee_vel_log)
    #plot_individual_samples(ee_vel_log)
    
    # Iterate through each sample and plot its trends
    for sample_index, sample in enumerate(ee_vel_log):
        plot_single_sample(sample, sample_index)
        print(f"Sample {sample_index+1} Velocity Data: {sample}")
        input("Press Enter to continue to the next sample...")