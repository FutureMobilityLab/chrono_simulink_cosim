import numpy as np
import matplotlib.pyplot as plt
from stiremod import StiremodTire

# --- Simulation and Plotting ---

# PARAMETERS
# IMPORTANT: Replace these with the values from your images. 
# These are representative placeholders for a P205/65R15 tire.
params = {
    'Fzt': 2403.0,       # Rated load (lbs)
    'Tw': 10.4,           # Tread width (inches)
    'Tp': 35.0,          # Tire Pressure (psi)
    
    # Peak Friction vs Load (Eq 13)
    'B1x': -4.7226e-4, 'B3x': 1.2688, 'B4x': 2.879e-7,
    'B1y': -1.9037e-4, 'B3y': 1.1947, 'B4y': 2.2025e-8,
    
    # Stiffness vs Load (Eq 4, 5)
    'A0': -1063.4, 'A1': 21.169, 'A2': 4794.7, 'Kx': 0.3014,
    'CS_FZ': 17.7325,       # Longitudinal stiffness slope (normalized)
    
    # Camber Stiffness (Eq 8)
    'A3': 0.87326, 'A4': 29825.0, 'K_gamma': 0.9,
    
    # Saturation Shape (Eq 6) - C parameters
    'C1': 0.6633, 'C2': 0.2184, 'C3': 0.4867, 'C4': 0.1622, 'C5': 1.2732,
    
    # Friction Decay (Eq 12)
    'K_mux': 0.303, 'K_muy': 0.45091 -7.9655e-5*1e3,
    
    # Misc
    'Ka': 0.0365,           # Patch length sensitivity (Eq 2)
    'K1': -1.1993e-4,        # Aligning stiffness slope (Eq 11)
    'G1': 1.3139,          # Aligning moment shape (Eq 10)
    'G2': 1.0,           # Combined slip moment factor (Eq 10)
    
    # Environmental
    'SN_o': 85, 'SN_t': 85
}

tire = StiremodTire(params)

# Set up test conditions
fz_test = 935.0  # Test load (lbs)

# 1. Sweep Alpha (for Fy and Mz)
alphas = np.linspace(-20, 20, 100)
fy_alpha = []
mz_alpha = []
for a in alphas:
    fx, fy, mz = tire.calculate(Fz=fz_test, alpha_deg=a, slip_ratio=0.0, gamma_deg=0.0)
    fy_alpha.append(fy)
    mz_alpha.append(mz)

# 2. Sweep Slip Ratio (for Fx)
slips = np.linspace(-1.0, 1.0, 100)
fx_slip = []
for s in slips:
    fx, fy, mz = tire.calculate(Fz=fz_test, alpha_deg=0.0, slip_ratio=s, gamma_deg=0.0)
    fx_slip.append(fx)

# 3. Friction Ellipse (Sweep Alpha at different Slip Ratios)
# We generate a grid of operating points
ellipse_fx = []
ellipse_fy = []
# Create a sweep of combined conditions
s_range = np.arange(0.0, 0.8, 0.01)
a_range = [-6, -4, -2, 2, 4, 6]
for a in a_range:
    fxi, fyi = [], []
    for s in s_range:
        fx, fy, mz = tire.calculate(Fz=fz_test, alpha_deg=a, slip_ratio=s, gamma_deg=0.0)
        fxi.append(fx)
        fyi.append(fy)
    ellipse_fx.append(fxi)
    ellipse_fy.append(fyi)
    
# --- Generating Plots ---
plt.figure(figsize=(12, 10))

# Plot 1: Fy vs Alpha
plt.subplot(2, 2, 1)
plt.plot(alphas, fy_alpha, 'b-', linewidth=2)
plt.title(f'Lateral Force vs. Slip Angle (Fz={fz_test} lbs)')
plt.xlabel('Slip Angle (deg)')
plt.ylabel('Lateral Force Fy (lbs)')
plt.grid(True)

# Plot 2: Mz vs Alpha
plt.subplot(2, 2, 2)
plt.plot(alphas, mz_alpha, 'r-', linewidth=2)
plt.title(f'Aligning Moment vs. Slip Angle (Fz={fz_test} lbs)')
plt.xlabel('Slip Angle (deg)')
plt.ylabel('Aligning Moment Mz (ft-lbs)')
plt.grid(True)

# Plot 3: Fx vs Slip
plt.subplot(2, 2, 3)
plt.plot(slips, fx_slip, 'g-', linewidth=2)
plt.title(f'Longitudinal Force vs. Slip Ratio (Fz={fz_test} lbs)')
plt.xlabel('Slip Ratio S')
plt.ylabel('Longitudinal Force Fx (lbs)')
plt.grid(True)

# Plot 4: Friction Ellipse
plt.subplot(2, 2, 4)
colors = ['k', 'b', 'm', 'c']
for i, s_val in enumerate(a_range):
    plt.plot(ellipse_fx[i], ellipse_fy[i], color=colors[i%len(colors)])
plt.title(f'Tire Force Interaction (Friction Ellipse)')
plt.xlabel('Longitudinal Force Fx (lbs)')
plt.ylabel('Lateral Force Fy (lbs)')
plt.legend()
plt.grid(True)
plt.axis('equal')

plt.tight_layout()
plt.show()