import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Read CSV file and extract timestamp and joint1 (positions[0]) values.
timestamps = []
joint1_positions = []

with open('joint_states.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        timestamps.append(float(row['timestamp']))
        joint1_positions.append(float(row['joint1']))

# Determine the sampling interval based on timestamps.
if len(timestamps) > 1:
    # Calculate average interval in milliseconds.
    avg_interval = ((timestamps[-1] - timestamps[0]) / (len(timestamps)-1)) * 1000
else:
    avg_interval = 100  # fallback value

# Create the plot.
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-', lw=2, label='Joint 1 Position')
# Add the reference position line at 0.0 (red dashed line).
ax.axhline(y=0.0, color='red', linestyle='--', lw=2, label='Reference (0.0)')
ax.set_xlim(timestamps[0], timestamps[-1])
y_min = min(joint1_positions) - 0.1
y_max = max(joint1_positions) + 0.1
ax.set_ylim(y_min, y_max)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Joint 1 Position (rad)')
ax.set_title('Joint 1 Position vs Time')
ax.legend()

# Initialization function: plot an empty line.
def init():
    line.set_data([], [])
    return line,

# Animation update function.
def update(frame):
    # Update the line with data up to the current frame.
    x = timestamps[:frame+1]
    y = joint1_positions[:frame+1]
    line.set_data(x, y)
    return line,

# Create the animation.
ani = animation.FuncAnimation(fig, update, frames=len(timestamps),
                              init_func=init, blit=True, interval=avg_interval)

# Save the animation as an MP4 video.
output_video = 'joint_position_video.mp4'
ani.save(output_video, writer='ffmpeg', fps=1000/avg_interval)
print(f"Animation saved as {output_video}")

plt.show()
