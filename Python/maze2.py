import serial
import json
import random
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

# Initialize serial communication with Arduino
arduino_port = 'COM6'  # Replace with your Arduino port
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

# Define turning actions only
actions = ["left", "right", "back"]

# Initialize Q-Table
state_space = (3, 3, 3, 4)  # Front, Right, Left (3 levels each), and Heading (4 directions)
q_table = np.zeros(state_space + (len(actions),))

# Learning parameters
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 1.0  # Exploration rate
epsilon_decay = 0.99
min_epsilon = 0.2

# Logging and monitoring data
rewards_per_episode = []
steps_per_episode = []
action_counts = {action: 0 for action in actions}  # Count actions for monitoring
visited_states = {}  # Tracks visit counts for each state

# New: Heatmap Data Storage
maze_heatmap = np.zeros(state_space)  # Now includes heading and all sensor discretizations

# New: Log Data Storage
session_data = {
    "episodes": [],
    "settings": {
        "alpha": alpha,
        "gamma": gamma,
        "epsilon_start": 1.0,
        "epsilon_decay": epsilon_decay,
        "min_epsilon": min_epsilon,
    },
}

def plot_heatmap():
    """
    Plot a heatmap for each heading to visualize state visit frequencies.
    Each heatmap is 2D: Front vs. Right or Front vs. Left, sliced by Heading.
    """
    headings = ["north", "east", "south", "west"]

    for heading_idx, heading in enumerate(headings):
        # Heatmap for Front vs. Right
        heatmap_right = maze_heatmap[:, :, :, heading_idx].sum(axis=1)  # Sum over Left dimension
        plt.figure(figsize=(8, 6))
        sns.heatmap(heatmap_right, annot=True, fmt=".0f", cmap="coolwarm", cbar=True)
        plt.title(f"Maze Heatmap: Front vs. Right (Heading: {heading.capitalize()})")
        plt.xlabel("Right Sensor Discretization")
        plt.ylabel("Front Sensor Discretization")
        plt.show()

        # Heatmap for Front vs. Left
        heatmap_left = maze_heatmap[:, :, :, heading_idx].sum(axis=0)  # Sum over Right dimension
        plt.figure(figsize=(8, 6))
        sns.heatmap(heatmap_left, annot=True, fmt=".0f", cmap="coolwarm", cbar=True)
        plt.title(f"Maze Heatmap: Front vs. Left (Heading: {heading.capitalize()})")
        plt.xlabel("Left Sensor Discretization")
        plt.ylabel("Front Sensor Discretization")
        plt.show()

# Append episode data to JSON log
def log_episode_data(episode, steps, total_reward, data):
    """
    Append the episode's data to the JSON log.
    """
    episode_entry = {
        "episode": episode,
        "steps": steps,
        "total_reward": total_reward,
        "data": data,
    }
    session_data["episodes"].append(episode_entry)


# Visualization
def plot_metrics():
    """
    Plot metrics to visualize training progress.
    """
    plt.figure(figsize=(12, 5))

    # Plot rewards
    plt.subplot(1, 2, 1)
    plt.plot(rewards_per_episode, label="Total Reward")
    plt.xlabel("Episode")
    plt.ylabel("Reward")
    plt.title("Total Reward per Episode")
    plt.grid()
    plt.legend()

    # Plot steps
    plt.subplot(1, 2, 2)
    plt.plot(steps_per_episode, label="Steps per Episode")
    plt.xlabel("Episode")
    plt.ylabel("Steps")
    plt.title("Steps per Episode")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

def discretize_state(sensor_data, heading):
    """
    Convert sensor data and heading into a discrete state for Q-table indexing.
    """
    def discretize_distance(distance):
        if distance > 100:
            return 2  # Far
        elif distance > 20:
            return 1  # Medium
        else:
            return 0  # Close

    front_obstacle, front_distance = sensor_data[0]
    right_obstacle, right_distance = sensor_data[1]
    left_obstacle, left_distance = sensor_data[2]

    # Discretize distances
    front = (front_obstacle, discretize_distance(front_distance))
    right = (right_obstacle, discretize_distance(right_distance))
    left = (left_obstacle, discretize_distance(left_distance))

    # Map heading to integer (north = 0, east = 1, south = 2, west = 3)
    heading_map = {"north": 0, "east": 1, "south": 2, "west": 3}
    heading_idx = heading_map[heading]

    # Return a discrete tuple state
    return (front[1], right[1], left[1], heading_idx)

def update_heading(current_heading, action):
    """
    Update the car's heading based on the action taken.
    """
    directions = ["north", "east", "south", "west"]
    idx = directions.index(current_heading)
    if action == "left":
        return directions[(idx - 1) % 4]  # Turn 90 degrees counterclockwise
    elif action == "right":
        return directions[(idx + 1) % 4]  # Turn 90 degrees clockwise
    elif action == "back":
        return directions[(idx + 2) % 4]  # Turn 180 degrees
    return current_heading  # Default to current heading

def get_sensor_data():
    """
    Receive and parse sensor data from Arduino.
    """
    line = ser.readline().decode('utf-8').strip()
    try:
        data = json.loads(line)  # Parse JSON data
        print("good")
        print(data)
        return (
            (data["Front"]["Obstacle"], data["Front"]["Distance"]),
            (data["Right"]["Obstacle"], data["Right"]["Distance"]),
            (data["Left"]["Obstacle"], data["Left"]["Distance"])
        )
    except (json.JSONDecodeError, KeyError):
        #print("bad")
        return None  # Return None if data is invalid

def send_action_to_arduino(action):
    """
    Send a movement command to Arduino.
    """
    command = {"left": "L", "right": "R", "back": "B"}[action]
    ser.write((command + "\n").encode())

def send_forward_to_arduino():
    """
    Continuously send forward command to Arduino.
    """
    ser.write("F\n".encode())

def get_reward(sensor_data, state):
    """
    Compute reward based on sensor data and revisiting states.
    """
    front, right, left = sensor_data[:3]
    state_visits = visited_states.get(state, 0)

    if front[0] == 1:  # Obstacle detected in front
        return -10
    elif right[0] == 1 and left[0] == 1:  # Dead-end scenario
        return -5
    elif state_visits > 1:  # Penalize revisiting states
        return -2 * state_visits
    elif front[0] == 0:  # Clear path ahead
        return 10
    return -1  # Default step penalty

# Training loop (updated)
try:
    ser.write("M\n".encode())
    episodes = 250
    log_file = open("training_log.txt", "w")  # File to save logs

    for episode in range(episodes):
        print(f"Episode {episode + 1}")
        state = get_sensor_data()  # Initial state from Arduino
        heading = "north"  # Assume starting heading is north (update dynamically in real use)
        if state is None:
            continue  # Skip if no valid data received

        discrete_state = discretize_state(state, heading)
        visited_states[discrete_state] = visited_states.get(discrete_state, 0) + 1
        maze_heatmap[discrete_state[:3]] += 1  # Update heatmap
        total_reward = 0
        steps = 0
        recent_states = []  # To detect loops
        episode_log = []  # Store per-step data for JSON logging

        while True:
            # Continuously send forward command
            send_forward_to_arduino()

            # Prioritized exploration
            if random.uniform(0, 1) < epsilon:
                # Prefer unexplored or less-visited actions
                unexplored_actions = [
                    a for a in actions if action_counts[a] < 10
                ]
                action = random.choice(unexplored_actions) if unexplored_actions else random.choice(actions)
            else:
                action = actions[np.argmax(q_table[discrete_state])]  # Exploit

            # Send turn action to Arduino
            send_action_to_arduino(action)

            # Update heading based on action
            heading = update_heading(heading, action)

            # Receive updated state
            new_state = get_sensor_data()
            if new_state is None:
                continue  # Skip if no valid data received

            # Discretize the new state
            discrete_new_state = discretize_state(new_state, heading)
            visited_states[discrete_new_state] = visited_states.get(discrete_new_state, 0) + 1
            maze_heatmap[discrete_new_state[:3]] += 1  # Update heatmap

            # Detect loops
            recent_states.append(discrete_new_state)
            if len(recent_states) > 10:
                recent_states.pop(0)  # Keep only the last 10 states
            if len(recent_states) == 10 and len(set(recent_states)) < 5:
                print("Loop detected. Terminating episode.")
                break

            # Calculate reward
            reward = get_reward(new_state, discrete_new_state)
            total_reward += reward

            # Update Q-value
            best_next_action = np.max(q_table[discrete_new_state])
            q_table[discrete_state + (actions.index(action),)] += alpha * (
                reward + gamma * best_next_action - q_table[discrete_state + (actions.index(action),)]
            )

            # Log data
            log_file.write(
                f"Episode: {episode + 1}, Step: {steps + 1}, State: {discrete_state}, Action: {action}, Reward: {reward}\n"
            )
            action_counts[action] += 1
            episode_log.append({
                "step": steps + 1,
                "state": discrete_state,
                "action": action,
                "reward": reward,
                "sensor_data": state,
            })

            # Transition to new state
            discrete_state = discrete_new_state
            steps += 1

            # Termination condition
            if reward == 10 or steps > 150:  # Goal reached or too many steps
                break

        # Decay epsilon
        epsilon = max(min_epsilon, epsilon * epsilon_decay)

        # Record metrics
        rewards_per_episode.append(total_reward)
        steps_per_episode.append(steps)

        # Log episode data for JSON
        log_episode_data(episode + 1, steps, total_reward, episode_log)

        print(f"Total Reward: {total_reward}")

except KeyboardInterrupt:
    print("\nTraining interrupted by user!")

finally:
    log_file.close()
    ser.close()
    print("\nPlotting metrics...")
    plot_metrics()
    print("\nPlotting heatmap...")
    #plot_heatmap()
    print("\nSaving session data to JSON...")
    with open("session_data.json", "w") as json_file:
        json.dump(session_data, json_file, indent=4)
    print("\nAction Distribution:")
    for action, count in action_counts.items():
        print(f"{action}: {count}")