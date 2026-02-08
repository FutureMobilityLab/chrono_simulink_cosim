#include "salaani_model.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <cassert>

namespace tire {

Forces SalaaniTireModel::calculateTireForces(const double ALPHA, const double S,
                                              const double GAMMA, const double FZ,
                                              const double MUNOM) {
  Forces forces;
  if (!is_valid) {
    std::cout << "Tire model is not valid, have you set parameters?\n";
    return forces;
  }

  // Cap vertical force to valid range.
  double Fz_clamped = std::min(tire_params.FzMax, FZ);

  // Limit slip ratio to avoid numerical issues
  const double S_clamped = std::clamp(S, -0.99, 0.99);
  double FZ2 = Fz_clamped * Fz_clamped;
  
  // Calculate friction ratio
  double MURATIO = MUNOM / tire_params.MUNTEST;

  // Calculate empirical physical properties

  // Pneumatic trail (Equation 29)
  double Tz = tire_params.tz1 * FZ2 + tire_params.tz2 * Fz_clamped;
  
  // Overturning moment arm (Equation 30)
  double Tx = tire_params.tx1 * FZ2 + tire_params.tx2 * Fz_clamped + tire_params.tx3;
    
  // Lateral stiffness (Equation 26)
  double CA = tire_params.Cam * (1.0 - std::exp(tire_params.C1 * std::pow(Fz_clamped/tire_params.FZCam, 2) + 
                                          tire_params.C2 * (Fz_clamped/tire_params.FZCam)));
  
  // Longitudinal stiffness (Equation 27)
  double CS = tire_params.Ckm * std::pow(Fz_clamped/tire_params.FZCKM, tire_params.n_val);
  
  // Inclination angle lateral force stiffness (Equation 31)
  double FYGAMMA = tire_params.Cr1 * Fz_clamped + tire_params.Cr2 * FZ2;
  
  // Determine reference load for friction calculations
  double FZ1 = (Fz_clamped < tire_params.FZ0) ? tire_params.FZ0 : Fz_clamped;
  
  // Longitudinal peak coefficient of friction (Equation 22)
  double MUXp = MURATIO * tire_params.mu_p0_long * 
                std::pow(FZ1/tire_params.FZ0, tire_params.eta2_long + tire_params.eta1_long * std::log(FZ1/tire_params.FZ0));
  
  // Lateral peak coefficient of friction (Equation 22)
  double MUYp = MURATIO * tire_params.mu_p0_lat * 
                std::pow(FZ1/tire_params.FZ0, tire_params.eta2_lat + tire_params.eta1_lat * std::log(FZ1/tire_params.FZ0));
        
  // Longitudinal decay of friction (Equation 23)
  double DMUx = tire_params.d1_long * FZ2 + tire_params.d2_long * Fz_clamped + tire_params.d3_long;
  
  // Lateral decay of friction (Equation 23)
  double DMUy = tire_params.d1_lat * FZ2 + tire_params.d2_lat * Fz_clamped + tire_params.d3_lat;
  
  // Adjust for plysteer
  const double alpha_mod = ALPHA - tire_params.plysteer;
  
  // Calculate combined slip magnitude (Equation 24)
  double Slip = std::min(1.0, std::sqrt(std::pow(std::sin(alpha_mod), 2) +
                                        std::pow(S_clamped * std::cos(alpha_mod), 2)));
  
  // Sliding coefficients of friction
  double MNUy = MUYp * (1.0 - DMUy * Slip) * tire_params.epsilon_sy;
  double MNUx = MUXp * (1.0 - DMUx * Slip) * tire_params.epsilon_xx;
    
  // Effect of slip on longitudinal stiffness
  double CSp = CS + (CA - CS) * Slip;
  
  // ============ Salaani's Model Core Calculations ============
  
  // Adhesion potential rate (Equation 11)
  double SIGMA = std::sqrt(std::pow(CA * std::tan(alpha_mod) / (1.0-S_clamped) / MUYp / Fz_clamped, 2) +
                          std::pow(CS * S_clamped / (1.0-S_clamped) / MUXp / Fz_clamped, 2));
      
  // Adhesion and sliding functions (Equations 18 and 19)
  double SIGMA2 = SIGMA * SIGMA;
  double SIGMAU = (1.0 - SIGMA2) / (1.0 + SIGMA2);
  double F_a = 4.0/PI * SIGMA / std::pow(SIGMA2 + 1.0, 2);
  double F_s = 1.0/PI * (PI/2.0 - SIGMAU * std::sqrt(1.0 - SIGMAU*SIGMAU) - std::asin(SIGMAU));
  
  // Calculate force scaling parameters
  double SIGMAm = std::sqrt(std::pow(CA * std::tan(alpha_mod) / MUYp, 2) +
                            std::pow(CS * S_clamped / MUXp, 2));
  double SIGMASm = std::sqrt(std::pow(CA * std::tan(alpha_mod) / MNUy, 2) +
                            std::pow(CSp * S_clamped / MNUx, 2));
    
  // Lateral and longitudinal forces (Equations 14 and 15)
  if (SIGMAm < 1.0e-6) {
      forces.FY = 0.0;
      forces.FX = 0.0;
  } else {
      forces.FY = Fz_clamped * CA * std::tan(alpha_mod) * (F_a/SIGMAm + F_s/SIGMASm);
      forces.FX = -Fz_clamped * S_clamped * (CS * F_a/SIGMAm + CSp * F_s/SIGMASm);
  }
  
  // Aligning torque (Equation 20)
  forces.MZ = Tz * CA * std::tan(alpha_mod) / std::pow(tire_params.m1 * SIGMA2 + tire_params.m0, 2) +
              forces.FY * tire_params.epsilon_x * F_s;
  
  // Overturning moment (Equation 21)
  forces.MX = -Tx * forces.FY;
  
  // Add camber effect to lateral force (Equation 32)
  forces.FY = forces.FY + FYGAMMA * GAMMA * (1.0 - F_s);

  // Adjust sign of forces to match plots in paper.
  forces.FY *= -1;
  forces.MZ *= -1;
  
  return forces;
}

void SalaaniTireModel::validateParameters() const {
  // Critical parameters that must be positive to avoid division by zero and invalid physics
  if (tire_params.MUNTEST <= 0.0) {
    throw std::invalid_argument("MUNTEST must be positive (test friction coefficient)");
  }
  if (tire_params.FZ0 <= 0.0) {
    throw std::invalid_argument("FZ0 must be positive (reference load for friction calculations)");
  }
  if (tire_params.FzMax <= 0.0) {
    throw std::invalid_argument("FzMax must be positive (maximum valid normal force)");
  }

  // Stiffness parameters should be positive
  if (tire_params.Cam <= 0.0) {
    throw std::invalid_argument("Cam must be positive (maximum lateral stiffness)");
  }
  if (tire_params.FZCam <= 0.0) {
    throw std::invalid_argument("FZCam must be positive (reference load for lateral stiffness)");
  }
  if (tire_params.Ckm <= 0.0) {
    throw std::invalid_argument("Ckm must be positive (initial longitudinal stiffness)");
  }
  if (tire_params.FZCKM <= 0.0) {
    throw std::invalid_argument("FZCKM must be positive (reference load for longitudinal stiffness)");
  }

  // Friction coefficients should be positive
  if (tire_params.mu_p0_lat <= 0.0) {
    throw std::invalid_argument("mu_p0_lat must be positive (lateral peak friction at reference load)");
  }
  if (tire_params.mu_p0_long <= 0.0) {
    throw std::invalid_argument("mu_p0_long must be positive (longitudinal peak friction at reference load)");
  }
}

} // namespace tire
