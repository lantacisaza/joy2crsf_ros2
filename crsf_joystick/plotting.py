import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("pid_log.csv")

# Create time array
time = list(range(len(df)))

# Convert columns to numpy arrays
x = df['x'].to_numpy()
y = df['y'].to_numpy()
z = df['z'].to_numpy()
roll = df['roll'].to_numpy()
pitch = df['pitch'].to_numpy()
throttle = df['throttle'].to_numpy()
yaw = df['yaw'].to_numpy()

# Plotting
plt.figure(figsize=(14, 16))

plt.subplot(3, 1, 1)
plt.plot(time, x)
plt.ylabel("X [m]")
plt.title("X Position")



plt.subplot(3, 1, 2)
plt.plot(time, pitch)
plt.ylabel("Pitch [deg]")
plt.title("Pitch Angle")

plt.subplot(3, 1, 3)
plt.plot(time, throttle)
plt.ylabel("Throttle")
plt.title("Throttle Signal")



plt.tight_layout()
plt.show()
