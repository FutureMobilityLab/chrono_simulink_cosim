import pandas as pd
import matplotlib.pyplot as plt

# Load Data
df_alpha = pd.read_csv('tire_data_alpha.csv')
df_slip = pd.read_csv('tire_data_slip.csv')
df_ellipse = pd.read_csv('tire_data_ellipse.csv')

plt.figure(figsize=(12, 10))

# Plot 1: Fy vs Alpha
plt.subplot(2, 2, 1)
plt.plot(df_alpha['Alpha_deg'], df_alpha['Fy_lbs'], 'b-', linewidth=2)
plt.title('Lateral Force vs. Slip Angle')
plt.xlabel('Slip Angle (deg)')
plt.ylabel('Fy (lbs)')
plt.grid(True)

# Plot 2: Mz vs Alpha
plt.subplot(2, 2, 2)
plt.plot(df_alpha['Alpha_deg'], df_alpha['Mz_ftlbs'], 'r-', linewidth=2)
plt.title('Aligning Moment vs. Slip Angle')
plt.xlabel('Slip Angle (deg)')
plt.ylabel('Mz (ft-lbs)')
plt.grid(True)

# Plot 3: Fx vs Slip
plt.subplot(2, 2, 3)
plt.plot(df_slip['Slip_Ratio'], df_slip['Fx_lbs'], 'g-', linewidth=2)
plt.title('Longitudinal Force vs. Slip Ratio')
plt.xlabel('Slip Ratio')
plt.ylabel('Fx (lbs)')
plt.grid(True)

# Plot 4: Friction Ellipse
plt.subplot(2, 2, 4)
colors = ['k', 'b', 'm', 'c']
ids = df_ellipse['Slip_ID'].unique()
for i, uid in enumerate(ids):
    subset = df_ellipse[df_ellipse['Slip_ID'] == uid]
    label = f"Slip={subset['Slip_Val'].iloc[0]}"
    plt.plot(subset['Fx_lbs'], subset['Fy_lbs'], color=colors[i%len(colors)], label=label)

plt.title('Tire Force Interaction (Friction Ellipse)')
plt.xlabel('Fx (lbs)')
plt.ylabel('Fy (lbs)')
plt.axis('equal')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('tire_plots.png')
print("Plots saved to tire_plots.png")