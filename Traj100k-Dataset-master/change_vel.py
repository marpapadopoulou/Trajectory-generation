import numpy as np
import pickle

def directional_non_stationary_noise(step, initial_std=0.1, std_increase=0.01, drift=0.01):
    """
    Generate non-stationary noise with a constant directional drift.
    
    :param step: Current simulation step.
    :param initial_std: Initial standard deviation of the noise.
    :param std_increase: Increment of the standard deviation per step.
    :param drift: Constant drift applied to the mean in one direction.
    :return: Noise value for the current step.
    """
    current_std = initial_std + step * std_increase
    current_mean = step * drift
    noise = np.random.normal(loc=current_mean, scale=current_std)
    return noise

def add_directional_non_stationary_noise(velocity, step, initial_std=0.1, std_increase=0.01, drift=0.01):
    noise = np.array([directional_non_stationary_noise(step, initial_std, std_increase, drift) for _ in range(velocity.shape[0])])
    velocity_with_noise = velocity + noise
    return velocity_with_noise

def load_and_modify_data(original_path, new_path, initial_std=0.02, std_increase=0.01, drift=0.01):
    with open(original_path, 'rb') as f:
        data = pickle.load(f)

    ee_vel_log = data['ee_vel_log']

    for i, trajectory in enumerate(ee_vel_log):
        for step, velocity in enumerate(trajectory):
            ee_vel_log[i][step] = add_directional_non_stationary_noise(np.array(velocity), step, initial_std, std_increase, drift)

    data['ee_vel_log'] = ee_vel_log

    with open(new_path, 'wb') as f:
        pickle.dump(data, f)

    print(f"Data saved to {new_path}")

if __name__ == '__main__':
    original_path = 'Traj20k-dataset-std0.1.json'  # Update with your original file path
    new_path = 'Traj20k-dataset-modified.json'  # Update with your desired new file path
    load_and_modify_data(original_path, new_path)
