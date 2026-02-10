#include "stiremod.h"
#include <iostream>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper for degrees to radians
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

StiremodTire::StiremodTire(const StiremodParams& params) : p(params) {}

StiremodTire::TireForces StiremodTire::calculate(double Fz, double alpha_deg, double slip_ratio, double gamma_deg, double V_mph) {
    
    // Convert inputs to consistent units (Imperial: ft, rads, lbs)
    double alpha = deg2rad(alpha_deg);
    double gamma = deg2rad(gamma_deg);
    double S = slip_ratio;

    // Avoid division by zero in slip calcs
    double epsilon = 1e-10;

    // --- 1. Static Tire Properties ---

    // Equation 3: Initial Patch Length (apo)
    // Note: Tw must be converted to ft, Tp to lbs/ft^2 (psf) for dimensional consistency
    double Tw_ft = p.Tw / 12.0;
    double Tp_psf = p.Tp * 144.0;
    double a_po = std::sqrt(Fz * p.Fzt) / (Tw_ft * Tp_psf);

    // Equation 13: Peak Coefficients of Friction (Mu_px, Mu_py)
    // Uses Skid Number ratio (SN_o / SN_T). Assuming 1.0 if not specified.
    // Avoid division by zero if SN_t is 0
    double sn_ratio = (p.SN_t != 0.0) ? (p.SN_o / p.SN_t) : 1.0;

    double mu_px = (p.B1x * Fz + p.B3x + p.B4x * std::pow(Fz, 2)) * sn_ratio;
    double mu_py = (p.B1y * Fz + p.B3y + p.B4y * std::pow(Fz, 2)) * sn_ratio;

    // --- 2. Stiffness Coefficients ---

    // Fx_est for Equation 4: Estimated longitudinal force
    // "F_Xest = (CS/FZ) * Fz * S" (from text below Eq 4)
    double Fx_est = p.CS_FZ * Fz * S;

    // Equation 4: Lateral Stiffness Coefficient (Ks)
    // Note: Using absolute value of Fx_est as implied by context of braking/cornering stiffness
    double term_A = p.A0 + p.A1 * Fz - (p.A1 / p.A2) * std::pow(Fz, 2);

    double term_Kx = p.Kx * (std::abs(Fx_est) / Fz);

    double Ks = (2.0 / std::pow(a_po, 2)) * (term_A + term_Kx);

    // Equation 5: Longitudinal Stiffness Coefficient (Kc)
    double Kc = (2.0 / std::pow(a_po, 2)) * Fz * p.CS_FZ;

    // --- 3. Composite Slip Calculation ---

    // Equation 2: Tire Contact Patch Length (ap)
    // Circular dependency: ap depends on Fx. Using Fx_est (linear approx) to resolve.
    double ap = a_po * (1.0 - p.Ka * (Fx_est / Fz));

    // Equation 1: Composite Slip (sigma)
    // Term inside the square root
    // Note: S/(1-S) handling. If S=1 (locked), this goes to infinity. 
    // Practical limit: clip S slightly below 1.0 for calculation.
    double S_clamped = std::max(-0.999, std::min(S, 0.999));
    double slip_term = std::pow(S_clamped / (1.0 - S_clamped), 2);

    double term_lat = (std::pow(Ks, 2) * std::pow(std::tan(alpha), 2)) / (std::pow(mu_py, 2) + epsilon);
    double term_long = (std::pow(Kc, 2) / (std::pow(mu_px, 2) + epsilon)) * slip_term;

    double sigma = (M_PI * std::pow(ap, 2) / (8.0 * Fz)) * std::sqrt(term_lat + term_long);

    // --- 4. Force Saturation ---

    // Equation 6: Force Saturation Function f(sigma)
    // Rational polynomial
    double num = p.C1 * std::pow(sigma, 3) + p.C2 * std::pow(sigma, 2) + p.C5 * sigma;
    double den = p.C1 * std::pow(sigma, 3) + p.C3 * std::pow(sigma, 2) + p.C4 * sigma + 1.0;
    double f_sigma = num / den;

    // --- 5. Transitions and Decay ---

    // Equation 14: Lateral/Longitudinal Stiffness Transition (Kc_prime)
    // Used for locked wheel symmetry
    double slip_geom = std::sqrt(std::pow(std::sin(alpha), 2) + std::pow(S, 2) * std::pow(std::cos(alpha), 2));
    double Kc_prime = Kc + (Ks - Kc) * slip_geom;

    // Equation 8: Camber Stiffness (Y_gamma)
    double Y_gamma = p.A3 * Fz - (p.A3 / p.A4) * std::pow(Fz, 2);

    // Equation 15: Camber Force Stiffness Transition (Y_gamma_prime)
    double Y_gamma_prime = Y_gamma * (1.0 - p.K_gamma * std::pow(f_sigma, 2));

    // Equation 12: Transition Coefficient of Friction (mu_x, mu_y)
    // Calculates the decay from Peak to Slide
    
    // Dynamic K_muy Calculation
    // Logic: K_muy = Offset + Slope * Fz (Clamped at max_Fz)
    double Fz_muy = std::min(Fz, p.K_muy_max_Fz);
    
    double K_muy_val = p.K_muy_offset + p.K_muy_slope * Fz_muy;

    double mu_x = mu_px * (1.0 - p.K_mux * slip_geom);
    double mu_y = mu_py * (1.0 - K_muy_val * slip_geom);

    // --- 6. Final Force Calculation ---

    double denom_force = std::sqrt(std::pow(Ks, 2) * std::pow(std::tan(alpha), 2) + std::pow(Kc, 2) * std::pow(S, 2)) + epsilon;

    // Equation 7: Normalized Side Force -> Fy
    // Fy = (mu_y * Fz) * [Directional Component] + CamberForce
    // Note: f(sigma) = Fc / (mu * Fz). 
    // We use f_sigma scaling the available friction mu_y.
    // NOTE: Added negative sign to match coordinate system where positive alpha -> negative Fy
    double fy_pneumatic = (-f_sigma * Ks * std::tan(alpha)) / denom_force;
    double Fy = (mu_y * Fz) * fy_pneumatic + Y_gamma_prime * gamma;

    // Equation 9: Normalized Longitudinal Force -> Fx
    // Note the negative sign convention for braking/slip
    double fx_pneumatic = (-f_sigma * Kc_prime * S) / denom_force;
    double Fx = (mu_x * Fz) * fx_pneumatic;

    // --- 7. Aligning Moment ---

    // Equation 11: Aligning Moment Stiffness (Km)
    double Km = p.K1 * Fz;

    // Equation 10: Aligning Moment (Mz)
    // Term 1: Decay with sigma
    double mz_term1 = (Km * std::pow(ap, 2) * std::tan(alpha)) / std::pow(1.0 + p.G1 * std::pow(sigma, 2), 2);

    // Term 2: Pneumatic trail and longitudinal offset interaction
    double mz_term2 = (Ks / 2.0) - p.G2 * Kc * (S_clamped / (1.0 - S_clamped)) * (2.0 + std::pow(sigma, 2));

    double Mz = mz_term1 * mz_term2;

    return {Fx, Fy, Mz, alpha};
}