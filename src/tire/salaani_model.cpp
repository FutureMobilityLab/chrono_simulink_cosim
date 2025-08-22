#include "salaani_model.h"

#include <cmath>
#include <algorithm>

TireForces SalaaniTireModel::calculateTireForces(
    const TireParameters& tire, 
    double ALPHA, double S, 
    double GAMMA, double FZ) {
    
  // Limit slip ratio to avoid numerical issues
  S = std::min(std::abs(S), 0.99) * (S >= 0 ? 1.0 : -1.0);
  double FZ2 = FZ * FZ;
  
  // Calculate friction ratio
  double MURATIO = tire.MUNOM / tire.MUNTEST;
  
  // Calculate empirical physical properties
  
  // Pneumatic trail (Equation 29)
  double Tz = tire.tz1 * FZ2 + tire.tz2 * FZ;
  
  // Overturning moment arm (Equation 30)
  double Tx = tire.tx1 * FZ2 + tire.tx2 * FZ + tire.tx3;
    
  // Lateral stiffness (Equation 26)
  double CA = tire.Cam * (1.0 - std::exp(tire.C1 * std::pow(FZ/tire.FZCam, 2) + 
                                          tire.C2 * (FZ/tire.FZCam)));
  
  // Longitudinal stiffness (Equation 27)
  double CS = tire.Ckm * std::pow(FZ/tire.FZCKM, tire.n_val);
  
  // Inclination angle lateral force stiffness (Equation 31)
  double FYGAMMA = tire.Cr1 * FZ + tire.Cr2 * FZ2;
  
  // Determine reference load for friction calculations
  double FZ1 = (FZ < tire.FZ0) ? tire.FZ0 : FZ;
  
  // Longitudinal peak coefficient of friction (Equation 22)
  double MUXp = MURATIO * tire.mu_p0_long * 
                std::pow(FZ1/tire.FZ0, tire.eta0_long + tire.eta1_long * std::log(FZ1/tire.FZ0));
  
  // Lateral peak coefficient of friction (Equation 22)
  double MUYp = MURATIO * tire.mu_p0_lat * 
                std::pow(FZ1/tire.FZ0, tire.eta2_lat + tire.eta1_lat * std::log(FZ1/tire.FZ0));
        
  // Longitudinal decay of friction (Equation 23)
  double DMUx = tire.d1_long * FZ2 + tire.d2_long * FZ + tire.d3_long;
  
  // Lateral decay of friction (Equation 23)
  double DMUy = tire.d1_lat * FZ2 + tire.d2_lat * FZ + tire.d3_lat;
  
  // Adjust for plysteer
  ALPHA = ALPHA - tire.plysteer;
  
  // Calculate combined slip magnitude (Equation 24)
  double Slip = std::min(1.0, std::sqrt(std::pow(std::sin(ALPHA), 2) + 
                                        std::pow(S * std::cos(ALPHA), 2)));
  
  // Sliding coefficients of friction
  double MNUy = MUYp * (1.0 - DMUy * Slip) * tire.epsilon_sy;
  double MNUx = MUXp * (1.0 - DMUx * Slip) * tire.epsilon_xx;
    
  // Effect of slip on longitudinal stiffness
  double CSp = CS + (CA - CS) * Slip;
  
  // ============ Salaani's Model Core Calculations ============
  
  // Adhesion potential rate (Equation 11)
  double SIGMA = std::sqrt(std::pow(CA * std::tan(ALPHA) / (1.0-S) / MUYp / FZ, 2) + 
                          std::pow(CS * S / (1.0-S) / MUXp / FZ, 2));
      
  // Adhesion and sliding functions (Equations 18 and 19)
  double SIGMA2 = SIGMA * SIGMA;
  double SIGMAU = (1.0 - SIGMA2) / (1.0 + SIGMA2);
  double F_a = 4.0/PI * SIGMA / std::pow(SIGMA2 + 1.0, 2);
  double F_s = 1.0/PI * (PI/2.0 - SIGMAU * std::sqrt(1.0 - SIGMAU*SIGMAU) - std::asin(SIGMAU));
  
  // Calculate force scaling parameters
  double SIGMAm = std::sqrt(std::pow(CA * std::tan(ALPHA) / MUYp, 2) + 
                            std::pow(CS * S / MUXp, 2));
  double SIGMASm = std::sqrt(std::pow(CA * std::tan(ALPHA) / MNUy, 2) + 
                            std::pow(CSp * S / MNUx, 2));
  
  TireForces forces;
    
  // Lateral and longitudinal forces (Equations 14 and 15)
  if (SIGMAm < 1.0e-6) {
      forces.FY = 0.0;
      forces.FX = 0.0;
  } else {
      forces.FY = FZ * CA * std::tan(ALPHA) * (F_a/SIGMAm + F_s/SIGMASm);
      forces.FX = -FZ * S * (CS * F_a/SIGMAm + CSp * F_s/SIGMASm);
  }
  
  // Aligning torque (Equation 20)
  forces.MZ = Tz * CA * std::tan(ALPHA) / std::pow(tire.m1 * SIGMA2 + tire.m0, 2) + 
              forces.FY * tire.epsilon_x * F_s;
  
  // Overturning moment (Equation 21)
  forces.MX = -Tx * forces.FY;
  
  // Add camber effect to lateral force (Equation 32)
  forces.FY = forces.FY + FYGAMMA * GAMMA * (1.0 - F_s);
  
  return forces;
}
