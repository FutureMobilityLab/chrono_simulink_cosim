import os
import pandas as pd
import matplotlib.pyplot as plt

# Unit conversion constants
N_TO_LBF = 0.224809  # Newtons to pound-force
NM_TO_LBFFT = 0.737562  # Newton-meters to pound-force feet

# Display units flag
display_imperial = True  # Set to True for imperial units, False for SI

# Unit labels based on display_imperial flag
force_unit = "lbf" if display_imperial else "N"
moment_unit = "lbf-ft" if display_imperial else "N-m"

# Read the CSV file
output_dir = os.path.join(os.getcwd(), "DEMO_OUTPUT", "TIRE_TEST_RIG")
data = pd.read_csv(os.path.join(output_dir, "tire_test_data.csv"))

# Convert units if display_imperial is True
if display_imperial:
    # Convert forces from N to lbf
    for col in ['force_x', 'force_y', 'force_z']:
        data[col] = data[col] * N_TO_LBF
    
    # Convert moments from N-m to lbf-ft
    for col in ['moment_x', 'moment_y', 'moment_z']:
        data[col] = data[col] * NM_TO_LBFFT

# Skip the first n seconds.
n = 5  # seconds
data = data[data['time'] > n]

# Display in Imperial.
display_imperial = True

# Create a figure with multiple subplots
fig = plt.figure(figsize=(15, 10))
fig.suptitle('Tire Test Rig Results', fontsize=16)

# Plot 1: Slip angles
ax1 = plt.subplot(2, 2, 1)
ax1.plot(data['time'], data['long_slip'], label='Longitudinal slip')
ax1.plot(data['time'], data['slip_angle'], label='Slip angle')
ax1.plot(data['time'], data['camber_angle'], label='Camber angle')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (deg)')
ax1.grid(True)
ax1.legend()
ax1.set_title('Kinematic Data')

# Plot 2: Forces
ax2 = plt.subplot(2, 2, 2)
ax2.plot(data['time'], data['force_x'], label='Fx', color='red')
ax2.plot(data['time'], data['force_y'], label='Fy', color='green')
ax2.plot(data['time'], data['force_z'], label='Fz', color='blue')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel(f'Force ({force_unit})')
ax2.grid(True)
ax2.legend()
ax2.set_title('Forces')

# Plot 3: Contact Points
ax3 = plt.subplot(2, 2, 3)
ax3.plot(data['time'], data['point_x'], label='X', color='red')
ax3.plot(data['time'], data['point_y'], label='Y', color='green')
ax3.plot(data['time'], data['point_z'], label='Z', color='blue')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Position (m)')  # Position always in meters
ax3.grid(True)
ax3.legend()
ax3.set_title('Contact Points')

# Plot 4: Moments
ax4 = plt.subplot(2, 2, 4)
ax4.plot(data['time'], data['moment_x'], label='Mx', color='red')
ax4.plot(data['time'], data['moment_y'], label='My', color='green')
ax4.plot(data['time'], data['moment_z'], label='Mz', color='blue')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel(f'Moment ({moment_unit})')
ax4.grid(True)
ax4.legend()
ax4.set_title('Moments')

# Adjust layout and display
plt.tight_layout()
# plt.show()

# Create XY plots for analysis
plt.figure(figsize=(15, 5))

# Plot lateral force vs slip angle
plt.subplot(1, 3, 1)
plt.plot(data['slip_angle'], data['force_y'], 'b.')
plt.xlabel('Slip Angle (deg)')
plt.ylabel(f'Lateral Force ({force_unit})')
plt.grid(True)
plt.title('Lateral Force vs Slip Angle')

# Plot longitudinal force vs longitudinal slip
plt.subplot(1, 3, 2)
plt.plot(data['long_slip'], data['force_x'], 'r.')
plt.xlabel('Longitudinal Slip')
plt.ylabel(f'Longitudinal Force ({force_unit})')
plt.grid(True)
plt.title('Longitudinal Force vs Slip')

# Plot aligning moment vs slip angle
plt.subplot(1, 3, 3)
plt.plot(data['slip_angle'], data['moment_z'], 'g.')
plt.xlabel('Slip Angle (deg)')
plt.ylabel(f'Aligning Moment ({moment_unit})')
plt.grid(True)
plt.title('Aligning Moment vs Slip Angle')

plt.tight_layout()
plt.show()

# Optional: Save some basic statistics
# stats = data.describe()
# print("\nBasic Statistics:")
# print(stats)
