import numpy as np
import matplotlib.pyplot as plt

class StiremodTire:
    def __init__(self, params):
        """
        Initialize the STIREMOD tire model with a dictionary of parameters.
        
        Parameters dictionary keys must match the coefficients in the paper 
        (e.g., 'A0', 'A1', 'C1', 'Tw', 'Tp', etc.).
        """
        self.p = params

    def calculate(self, Fz, alpha_deg, slip_ratio, gamma_deg, V_mph=40.0):
        """
        Calculate tire forces and moments.
        
        Inputs:
            Fz: Normal load (lbs)
            alpha_deg: Slip angle (degrees)
            slip_ratio: Longitudinal slip ratio (S)
            gamma_deg: Camber angle (degrees)
            V_mph: Vehicle speed (mph) - used for lag, defaults to 40 for static plots
        
        Returns:
            Fx, Fy, Mz, alpha_rad
        """
        # Convert inputs to consistent units (Imperial: ft, rads, lbs)
        alpha = np.deg2rad(alpha_deg)
        gamma = np.deg2rad(gamma_deg)
        S = slip_ratio
        
        # Avoid division by zero in slip calcs
        epsilon = 1e-10
        
        # --- 1. Static Tire Properties ---
        
        # Equation 3: Initial Patch Length (apo)
        # Note: Tw must be converted to ft, Tp to lbs/ft^2 (psf) for dimensional consistency
        Tw_ft = self.p['Tw'] / 12.0
        Tp_psf = self.p['Tp'] * 144.0
        a_po = np.sqrt(Fz * self.p['Fzt']) / (Tw_ft * Tp_psf)
        
        # Equation 13: Peak Coefficients of Friction (Mu_px, Mu_py)
        # Uses Skid Number ratio (SN_o / SN_T). Assuming 1.0 if not specified.
        sn_ratio = self.p.get('SN_o', 85) / self.p.get('SN_t', 85)
        
        mu_px = (self.p['B1x'] * Fz + self.p['B3x'] + self.p['B4x'] * Fz**2) * sn_ratio
        mu_py = (self.p['B1y'] * Fz + self.p['B3y'] + self.p['B4y'] * Fz**2) * sn_ratio
        
        # --- 2. Stiffness Coefficients ---
        
        # Fx_est for Equation 4: Estimated longitudinal force
        # "F_Xest = (CS/FZ) * Fz * S" (from text below Eq 4)
        Fx_est = self.p['CS_FZ'] * Fz * S
        
        # Equation 4: Lateral Stiffness Coefficient (Ks)
        # Note: Using absolute value of Fx_est as implied by context of braking/cornering stiffness
        term_A = self.p['A0'] + self.p['A1'] * Fz - (self.p['A1'] / self.p['A2']) * Fz**2
        term_Kx = self.p['Kx'] * (np.abs(Fx_est) / Fz)
        Ks = (2 / a_po**2) * (term_A + term_Kx)
        
        # Equation 5: Longitudinal Stiffness Coefficient (Kc)
        Kc = (2 / a_po**2) * Fz * self.p['CS_FZ']
        
        # --- 3. Composite Slip Calculation ---
        
        # Equation 2: Tire Contact Patch Length (ap)
        # Circular dependency: ap depends on Fx. Using Fx_est (linear approx) to resolve.
        ap = a_po * (1 - self.p['Ka'] * (Fx_est / Fz))
        
        # Equation 1: Composite Slip (sigma)
        # Term inside the square root
        # Note: S/(1-S) handling. If S=1 (locked), this goes to infinity. 
        # Practical limit: clip S slightly below 1.0 for calculation.
        S_clamped = np.clip(S, -0.999, 0.999) 
        slip_term = (S_clamped / (1 - S_clamped))**2
        
        term_lat = (Ks**2 * np.tan(alpha)**2) / (mu_py**2 + epsilon)
        term_long = (Kc**2 / (mu_px**2 + epsilon)) * slip_term
        
        sigma = (np.pi * ap**2 / (8 * Fz)) * np.sqrt(term_lat + term_long)
        
        # --- 4. Force Saturation ---
        
        # Equation 6: Force Saturation Function f(sigma)
        # Rational polynomial
        num = self.p['C1'] * sigma**3 + self.p['C2'] * sigma**2 + self.p['C5'] * sigma
        den = self.p['C1'] * sigma**3 + self.p['C3'] * sigma**2 + self.p['C4'] * sigma + 1
        f_sigma = num / den
        
        # --- 5. Transitions and Decay ---
        
        # Equation 14: Lateral/Longitudinal Stiffness Transition (Kc_prime)
        # Used for locked wheel symmetry
        slip_geom = np.sqrt(np.sin(alpha)**2 + S**2 * np.cos(alpha)**2)
        Kc_prime = Kc + (Ks - Kc) * slip_geom
        
        # Equation 8: Camber Stiffness (Y_gamma)
        Y_gamma = self.p['A3'] * Fz - (self.p['A3'] / self.p['A4']) * Fz**2
        
        # Equation 15: Camber Force Stiffness Transition (Y_gamma_prime)
        Y_gamma_prime = Y_gamma * (1 - self.p['K_gamma'] * f_sigma**2)
        
        # Equation 12: Transition Coefficient of Friction (mu_x, mu_y)
        # Calculates the decay from Peak to Slide
        mu_x = mu_px * (1 - self.p['K_mux'] * slip_geom)
        mu_y = mu_py * (1 - self.p['K_muy'] * slip_geom)
        
        # --- 6. Final Force Calculation ---
        
        denom_force = np.sqrt(Ks**2 * np.tan(alpha)**2 + Kc**2 * S**2) + epsilon
        
        # Equation 7: Normalized Side Force -> Fy
        # Fy = (mu_y * Fz) * [Directional Component] + CamberForce
        # Note: f(sigma) = Fc / (mu * Fz). 
        # We use f_sigma scaling the available friction mu_y.
        fy_pneumatic = (-f_sigma * Ks * np.tan(alpha)) / denom_force  + Y_gamma_prime * gamma
        Fy = (mu_y * Fz) * fy_pneumatic
        
        # Equation 9: Normalized Longitudinal Force -> Fx
        # Note the negative sign convention for braking/slip
        fx_pneumatic = (-f_sigma * Kc_prime * S) / denom_force
        Fx = (mu_x * Fz) * fx_pneumatic
        
        # --- 7. Aligning Moment ---
        
        # Equation 11: Aligning Moment Stiffness (Km)
        Km = self.p['K1'] * Fz
        
        # Equation 10: Aligning Moment (Mz)
        # Term 1: Decay with sigma
        mz_term1 = (Km * ap**2 * np.tan(alpha)) / ((1 + self.p['G1'] * sigma**2)**2)
        
        # Term 2: Pneumatic trail and longitudinal offset interaction
        mz_term2 = (Ks / 2) - self.p['G2'] * Kc * (S_clamped / (1 - S_clamped)) * (2 + sigma**2)
        
        Mz = mz_term1 * mz_term2
        
        return Fx, Fy, Mz