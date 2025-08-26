// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: [Your Name]
// =============================================================================
//
// Implementation of the Salaani tire model for Project Chrono.
//
// The Salaani tire model is based on the unified tire model developed by
// Salaani and is capable of handling combined slip conditions.
//
// References:
// - Salaani, M. K. (2007). "Analytical Tire Forces and Moments Model with 
//   Validated Data." SAE Technical Paper 2007-01-0816.
// ===================================================================================

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "chrono/core/ChGlobal.h"
#include "chrono/functions/ChFunctionSineStep.h"
#include "chrono/utils/ChConstants.h"

#include "src/tire/ChSalaaniTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace chrono {
namespace vehicle {

namespace {

constexpr double LBF_TO_N = 4.4482216153;
constexpr double FTLBF_TO_NM = 1.3558179483;

} // namespace

// Helper function that replaces the erroneous ChFunctionSineStep::Eval().
// This uses half the sine wave to interpolate between two points: P1=(x1, y1) 
// and P2=(x2,y2). This is effectively mapping the input to the sine wave range:
// (-pi/2, pi/2) and then shifting and scaling to the desired output range.
double SineInterpolation(double x, double x1, double y1, double x2, double y2) {
  if (x <= x1) {
    return y1;
  }

  if (x >= x2) {
    return y2;
  }

  // map x in (x1, x2) to (0, 1).
  double x_mapped = (x - x1) / (x2 - x1);

  // map x in (0, 1) to (-pi/2, pi/2).
  double x_sin_range = x_mapped * CH_PI - CH_PI / 2;

  // map y in (-1, 1) to (0, 1).
  double y_mapped = (std::sin(x_sin_range) + 1)/2;

  // map y in (0, 1) to (y1, y2).
  return y1 + (y2 - y1) * y_mapped;
}

ChSalaaniTire::ChSalaaniTire(const std::string& name)
    : ChForceElementTire(name),
      m_vnum(0.01),
      m_begin_start_transition(0.0),
      m_end_start_transition(0.25),
      m_use_startup_transition(false),
      // m_vcoulomb(1.0),
      m_frblend_begin(0.1),
      m_frblend_end(1.0),
      m_bottom_radius(0.0),
      m_bottom_stiffness(0.0),
      m_rolling_resistance(0.01) {
    m_tireforce.force = ChVector3d(0, 0, 0);
    m_tireforce.point = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);

    m_salaani_coeff.FZ0 = 0.0;
    m_salaani_coeff.MUNOM = 0.8;
    m_salaani_coeff.MUNTEST = 0.8;
}

// -----------------------------------------------------------------------------

void ChSalaaniTire::Initialize(std::shared_ptr<ChWheel> wheel) {
  ChTire::Initialize(wheel);

  SetSalaaniParams();

  // Initialize contact patch state variables to 0
  m_states.slip_ratio = 0;
  m_states.alpha = 0;
  m_states.vta = m_vnum;
  // m_states.R_eff = m_unloaded_radius;

  // Initialize vertical damping parameters.
  double sidewall_height = m_width * m_aspect_ratio / 100;
  m_unloaded_radius = m_rim_radius + sidewall_height;
  // std::cout << "Set unloaded radius: " << m_unloaded_radius << "\n";
  m_states.R_eff = m_unloaded_radius;
  double deflection_at_load = 0.16 * sidewall_height;
  double vertical_stiffness = m_max_load * 9.81 / deflection_at_load;
  // double vertical_stiffness = 5e3;
  // std::cout << "Set vertical stiffness: " << vertical_stiffness << "\n";
  double damping_ratio = 0.9;  // Critical damping ratio
  m_vert_damping = 2.0 * damping_ratio * std::sqrt(vertical_stiffness * GetTireMass());
  // std::cout << "Set vertical damping: " << m_vert_damping << "\n";
  
  SetVerticalStiffness(vertical_stiffness);

  // Set bottoming parameters if not set
  if (m_bottom_radius == 0) {
    m_bottom_radius = m_rim_radius + 0.01;
  }
  if (m_bottom_stiffness == 0.0) {
      m_bottom_stiffness = 5.0 * m_d1;
  }
  
  // Build the lookup table for penetration depth as function of intersection area
  // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
  ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

  this->SetCollisionType(ChTire::CollisionType::SINGLE_POINT); // This doesn't use m_areaDep.
}

// -----------------------------------------------------------------------------

void ChSalaaniTire::Synchronize(double time, const ChTerrain& terrain) {
  m_time = time;
  WheelState wheel_state = m_wheel->GetState();

  // Extract the wheel normal (expressed in global frame)
  ChMatrix33<> A(wheel_state.rot);
  ChVector3d disc_normal = A.GetAxisY();

  // Check contact with terrain
  float mu_road;
  m_data.in_contact = DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                            m_width, m_areaDep, m_data.frame, m_data.depth, mu_road);
  // if (m_data.in_contact) {
    // std::cout << "DiscTerrainCollision args: " << terrain.GetHeight(wheel_state.pos) << ", "
    //           << wheel_state.pos << ", " << disc_normal << ", " << m_unloaded_radius << ", " << m_width << "\n";
  // }
  ChClampValue(mu_road, 0.1f, 1.0f);

  // The Salaani model computes MURATIO to scale peak friction. This is 
  // effectively the same but is used in the Dahl tire model for low speeds.
  m_states.muscale = mu_road / m_salaani_coeff.MUNTEST;

  // Calculate tire kinematics
  CalculateKinematics(wheel_state, m_data.frame);

  // Verify FZ0.
  if (m_salaani_coeff.FZ0 <= 0.0) {
    std::cerr << "FATAL error in ChSalaaniTire::Synchronize - Reference load FZ0 has not been set!" << std::endl;
    throw std::runtime_error("FATAL error in ChSalaaniTire::Synchronize - Reference load FZ0 has not been set!");
  }

  m_states.gamma = GetCamberAngle();

  if (m_data.in_contact) {
    // Wheel velocity in the ISO-C Frame
    ChVector3d vel = wheel_state.lin_vel;
    m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

    // Generate normal contact force. If the resulting force is negative, the disc
    // is moving away from the terrain so fast that no contact force is generated.
    // The sign of the velocity term in the damping function is negative since a
    // positive velocity means a decreasing depth, not an increasing depth.
    double Fn_stiff = GetNormalStiffnessForce(m_data.depth);
    double Fn_damp = GetNormalDampingForce(m_data.depth, -m_data.vel.z());
    double Fn_mag = Fn_stiff + Fn_damp;
    // std::cout << "Fn_stiff: " << Fn_stiff << "\n";
    // std::cout << "Fn_damp: " << Fn_damp << "\n";

    // Skip force calculations when the normal force = 0
    if (Fn_mag < 0) {
        Fn_mag = 0;
        m_data.in_contact = false;
    }

    m_data.normal_force = Fn_mag;
    double r_stat = m_unloaded_radius - m_data.depth;
    m_states.omega = wheel_state.omega;
    m_states.R_eff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
    m_states.vta = m_states.R_eff * std::abs(m_states.omega) + m_vnum;
    m_states.vsx = m_data.vel.x() - m_states.omega * m_states.R_eff;
    m_states.vsy = m_data.vel.y();
      
    // Calculate slip ratio and slip angle
    m_states.slip_ratio = -m_states.vsx / m_states.vta;
    m_states.alpha = atan(-m_states.vsy / m_states.vta);
      
    m_states.disc_normal = disc_normal;
  } else {
    // Reset all states if the tire comes off the ground
    m_data.normal_force = 0;
    m_states.R_eff = m_unloaded_radius;
    m_states.slip_ratio = 0;
    m_states.alpha = 0;
    m_states.vta = m_vnum;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.brx = 0;
    m_states.bry = 0;
    m_states.disc_normal = ChVector3d(0, 0, 0);
  }
}

void ChSalaaniTire::Advance(double step) {
  // Set tire forces to zero
  m_tireforce.force = ChVector3d(0, 0, 0);
  m_tireforce.moment = ChVector3d(0, 0, 0);

  // Return if no contact
  if (!m_data.in_contact)
      return;

  // Limit the effect of Fz on handling forces and torques to avoid nonsensical extrapolation of the curve
  // coefficients
  // m_data.normal_force is nevertheless still taken as the applied vertical tire force
  double Fz = std::min(m_data.normal_force, m_max_load);

  double Fx = 0, Fy = 0, Mz = 0, Mx = 0;
  double Fx0, Fy0;

  // Calculate low-speed Coulomb forces (Dahl model)
  CombinedCoulombForces(Fx0, Fy0, Fz, m_states.muscale);

  // Blending factor between Dahl and Salaani models
  double frblend = SineInterpolation(m_data.vel.x(), m_frblend_begin, 0.0, m_frblend_end, 1.0);

  // Calculate Salaani forces
  CalculateSalaaniForces(Fx, Fy, Mz, Mx, m_states.alpha, m_states.slip_ratio, 
                        m_states.gamma, m_data.normal_force / LBF_TO_N);

  // Account for unit conversion and sign convention.
  // Fx = std::clamp(Fx / LBF_TO_N, -1e6, 1e6);
  Fx *= LBF_TO_N;
  Fy *= LBF_TO_N;
  Mz *= FTLBF_TO_NM;
  Mx *= FTLBF_TO_NM;
  // Fx = 0;
  // Fy = 0;
  // Mz = 0;
  // Mx = 0;

  // Blend between Dahl and Salaani models
  // frblend = 0.0;
  Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
  Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

  // Calculate rolling resistance moment
  double My = -m_rolling_resistance * m_data.normal_force * m_unloaded_radius * tanh(m_states.omega);
  // double My = 0;

  // Apply startup transition if enabled
  double startup = 1;
  if (m_use_startup_transition) {
      startup = SineInterpolation(m_time, m_begin_start_transition, 0.0, m_end_start_transition, 1.0);
  }

  // Compile the force and moment vectors
  m_tireforce.force = ChVector3d(startup * Fx, startup * Fy, m_data.normal_force);
  m_tireforce.moment = startup * ChVector3d(Mx, My, Mz);

  // std::cout << "m_data: depth:" << m_data.depth << ", in_contact:" << m_data.in_contact << ", normal_force:" 
            // << m_data.normal_force << ", vel: " << m_data.vel << ".\n";
  // std::cout << "Force: " << m_tireforce.force.x() << ", " << m_tireforce.force.y() << ", " << m_tireforce.force.z() << ".\n";
  // printMyTireStates();
}

void ChSalaaniTire::CalculateSalaaniForces(double& fx, double& fy, double& mz, double& mx,
                                          double alpha, double slip_ratio, double gamma, double fz) {
  // Limit slip ratio to avoid numerical issues
  slip_ratio = std::min(std::abs(slip_ratio), 0.99) * (slip_ratio >= 0 ? 1.0 : -1.0);
  double fz2 = fz * fz;

  // Calculate friction ratio
  double MURATIO = m_salaani_coeff.MUNOM / m_salaani_coeff.MUNTEST;

  // Calculate empirical physical properties
  
  // Pneumatic trail (Equation 29)
  double Tz = m_salaani_coeff.tz1 * fz2 + m_salaani_coeff.tz2 * fz;
  
  // Overturning moment arm (Equation 30)
  double Tx = m_salaani_coeff.tx1 * fz2 + m_salaani_coeff.tx2 * fz + m_salaani_coeff.tx3;
  
  // Lateral stiffness (Equation 26)
  double CA = m_salaani_coeff.Cam * (1.0 - std::exp(m_salaani_coeff.C1 * std::pow(fz/m_salaani_coeff.FZCam, 2) + 
                                          m_salaani_coeff.C2 * (fz/m_salaani_coeff.FZCam)));
    
  // Longitudinal stiffness (Equation 27)
  double CS = m_salaani_coeff.Ckm * std::pow(fz/m_salaani_coeff.FZCKM, m_salaani_coeff.n_val);
  
  // Inclination angle lateral force stiffness (Equation 31)
  double FYGAMMA = m_salaani_coeff.Cr1 * fz + m_salaani_coeff.Cr2 * fz2;
  
  // Determine reference load for friction calculations
  double FZ1 = (fz < m_salaani_coeff.FZ0) ? m_salaani_coeff.FZ0 : fz;
  
  // Longitudinal peak coefficient of friction (Equation 22)
  double MUXp = MURATIO * m_salaani_coeff.mu_p0_long * 
                std::pow(FZ1/m_salaani_coeff.FZ0, m_salaani_coeff.eta0_long + m_salaani_coeff.eta1_long * std::log(FZ1/m_salaani_coeff.FZ0));
  
  // Lateral peak coefficient of friction (Equation 22)
  double MUYp = MURATIO * m_salaani_coeff.mu_p0_lat * 
                std::pow(FZ1/m_salaani_coeff.FZ0, m_salaani_coeff.eta2_lat + m_salaani_coeff.eta1_lat * std::log(FZ1/m_salaani_coeff.FZ0));
  
  // Longitudinal decay of friction (Equation 23)
  double DMUx = m_salaani_coeff.d1_long * fz2 + m_salaani_coeff.d2_long * fz + m_salaani_coeff.d3_long;
  
  // Lateral decay of friction (Equation 23)
  double DMUy = m_salaani_coeff.d1_lat * fz2 + m_salaani_coeff.d2_lat * fz + m_salaani_coeff.d3_lat;
    
  // Adjust for plysteer
  alpha = alpha - m_salaani_coeff.plysteer;
  
  // Calculate combined slip magnitude (Equation 24)
  double Slip = std::min(1.0, std::sqrt(std::pow(std::sin(alpha), 2) + 
                                        std::pow(slip_ratio * std::cos(alpha), 2)));
  
  // Sliding coefficients of friction
  double MNUy = MUYp * (1.0 - DMUy * Slip) * m_salaani_coeff.epsilon_sy;
  double MNUx = MUXp * (1.0 - DMUx * Slip) * m_salaani_coeff.epsilon_xx;
  
  // Effect of slip on longitudinal stiffness
  double CSp = CS + (CA - CS) * Slip;
  
  // ============ Salaani's Model Core Calculations ============
  
  // Adhesion potential rate (Equation 11)
  double SIGMA = std::sqrt(std::pow(CA * std::tan(alpha) / (1.0-slip_ratio) / MUYp / fz, 2) + 
                            std::pow(CS * slip_ratio / (1.0-slip_ratio) / MUXp / fz, 2));

  // Adhesion and sliding functions (Equations 18 and 19)
  double SIGMA2 = SIGMA * SIGMA;
  double SIGMAU = (1.0 - SIGMA2) / (1.0 + SIGMA2);
  double F_a = 4.0/CH_PI * SIGMA / std::pow(SIGMA2 + 1.0, 2);
  double F_s = 1.0/CH_PI * (CH_PI/2.0 - SIGMAU * std::sqrt(1.0 - SIGMAU*SIGMAU) - std::asin(SIGMAU));
  
  // Calculate force scaling parameters
  double SIGMAm = std::sqrt(std::pow(CA * std::tan(alpha) / MUYp, 2) + 
                            std::pow(CS * slip_ratio / MUXp, 2));
  double SIGMASm = std::sqrt(std::pow(CA * std::tan(alpha) / MNUy, 2) + 
                              std::pow(CSp * slip_ratio / MNUx, 2));
  
  // Lateral and longitudinal forces (Equations 14 and 15)
  if (SIGMAm < 1.0e-6) {
      fy = 0.0;
      fx = 0.0;
  } else {
      fy = fz * CA * std::tan(alpha) * (F_a/SIGMAm + F_s/SIGMASm);
      fx = -fz * slip_ratio * (CS * F_a/SIGMAm + CSp * F_s/SIGMASm);
  }
    
  // Aligning torque (Equation 20)
  mz = Tz * CA * std::tan(alpha) / std::pow(m_salaani_coeff.m1 * SIGMA2 + m_salaani_coeff.m0, 2) + 
        fy * m_salaani_coeff.epsilon_x * F_s;
  
  // Overturning moment (Equation 21)
  mx = -Tx * fy;
  
  // Add camber effect to lateral force (Equation 32)
  fy = fy + FYGAMMA * gamma * (1.0 - F_s);
}

void ChSalaaniTire::CombinedCoulombForces(double& fx, double& fy, double fz, double muscale) {
  ChVector2d F;

  double fc = fz * muscale;
  double h = this->m_stepsize;
    
  // Longitudinal friction force
  double brx_dot = m_states.vsx - m_dahl_coeff.sigma0 * m_states.brx * fabs(m_states.vsx) / fc;
  F.x() = -(m_dahl_coeff.sigma0 * m_states.brx + m_dahl_coeff.sigma1 * brx_dot);
  
  // Lateral friction force
  double bry_dot = m_states.vsy - m_dahl_coeff.sigma0 * m_states.bry * fabs(m_states.vsy) / fc;
  F.y() = -(m_dahl_coeff.sigma0 * m_states.bry + m_dahl_coeff.sigma1 * bry_dot);
  
  // Calculate the new ODE states (implicit Euler)
  m_states.brx = (fc * m_states.brx + fc * h * m_states.vsx) / (fc + h * m_dahl_coeff.sigma0 * fabs(m_states.vsx));
  m_states.bry = (fc * m_states.bry + fc * h * m_states.vsy) / (fc + h * m_dahl_coeff.sigma0 * fabs(m_states.vsy));

  // Combine forces (friction circle)
  if (F.Length() > fz * muscale) {
      F.Normalize();
      F *= fz * muscale;
  }
  fx = F.x();
  fy = F.y();
}

// -----------------------------------------------------------------------------

double ChSalaaniTire::GetNormalStiffnessForce(double depth) const {
    double F = depth * m_d1 + depth * depth * m_d2;  // tire force
    double free_depth = m_unloaded_radius - m_bottom_radius;
    if (depth - free_depth > 0) {
        // std::cout << "At bottom, applying bottom_stiffness: " << m_bottom_stiffness << "\n"; 
        F += (depth - free_depth) * m_bottom_stiffness;  // add bottom contact force
    }
    return F;
}

double ChSalaaniTire::GetNormalDampingForce(double depth, double velocity) const {
  return m_vert_damping * velocity;
}

// -----------------------------------------------------------------------------

// void ChSalaaniTire::SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc) {
//     // Calculate polynomial coefficients from test data [m],[N/m]
//     Eigen::MatrixXd A(defl.size(), 2);
//     Eigen::VectorXd b(defl.size());
//     Eigen::Vector2d r;
    
//     for (int k = 0; k < defl.size(); k++) {
//         A(k, 0) = defl[k];
//         A(k, 1) = defl[k] * defl[k];
//         b(k) = frc[k];
//     }
//     r = A.colPivHouseholderQr().solve(b);
//     m_d1 = r(0);
//     m_d2 = r(1);
    
//     if (m_verbose)
//         std::cout << "Stiffness Coeffs from test data d1 = " << m_d1 << "  d2 = " << m_d2 << "\n";
// }

// double ChSalaaniTire::GetTireMaxLoad(unsigned int li) {
//     double Weight_per_Tire[] = {
//         45,    46.5,  47.5,   48.7,   50,     51.5,   53,     54.5,   56,     58,     60,     61.5,   63,     65,
//         67,    69,    71,     73,     75,     77.5,   80.0,   82.5,   85.0,   87.5,   90.0,   92.5,   95.0,   97.5,
//         100.0, 103,   106,    109,    112,    115,    118,    121,    125,    128,    132,    136,    140,    145,
//         150,   155,   160,    165,    170,    175,    180,    185,    190,    195,    200,    206,    212,    218,
//         224,   230,   236,    243,    250,    257,    265,    272,    280,    290,    300,    307,    315,    325,
//         335,   345,   355,    365,    375,    387,    400,    412,    425,    437,    450,    462,    475,    487,
//         500,   515,   530,    545,    560,    580,    600,    615,    630,    650,    670,    690,    710,    730,
//         750,   775,   800,    825,    850,    875,    900,    925,    950,    975,    1000,   1030,   1060,   1090,
//         1120,  1150,  1180,   1215,   1250,   1285,   1320,   1360,   1400,   1450,   1500,   1550,   1600,   1650,
//         1700,  1750,  1800,   1850,   1900,   1950,   2000,   2060,   2120,   2180,   2240,   2300,   2360,   2430,
//         2500,  2575,  2650,   2725,   2800,   2900,   3000,   3075,   3150,   3250,   3350,   3450,   3550,   3650,
//         3750,  3875,  4000,   4125,   4250,   4375,   4500,   4625,   4750,   4875,   5000,   5150,   5300,   5450,
//         5600,  5850,  6000,   6150,   6300,   6500,   6700,   6900,   7100,   7300,   7500,   7750,   8000,   8250,
//         8500,  8750,  9000,   9250,   9500,   9750,   10000,  10300,  10600,  10900,  11200,  11500,  11800,  12150,
//         12500, 12850, 13200,  13600,  14000,  14500,  15000,  15550,  16000,  16500,  17000,  17500,  18000,  18500,
//         19000, 19500, 20000,  20600,  21200,  21800,  22400,  23000,  23600,  24300,  25000,  25750,  26500,  27250,
//         28000, 29000, 30000,  30750,  31500,  32500,  33500,  34500,  35500,  36500,  37500,  38750,  40000,  41250,
//         42500, 43750, 45000,  46250,  47500,  48750,  50000,  51500,  53000,  54500,  56000,  58000,  60000,  61500,
//         63000, 65000, 67000,  69000,  71000,  73000,  75000,  77500,  80000,  82500,  85000,  87500,  90000,  92500,
//         95000, 97500, 100000, 103000, 106000, 109000, 112000, 115000, 118000, 121000, 125000, 128500, 132000, 136000};

//     unsigned int nw = sizeof(Weight_per_Tire) / sizeof(double);
//     const double g = 9.81;
//     double fmax;
//     if (li < nw) {
//         fmax = Weight_per_Tire[li] * g;
//     } else {
//         fmax = Weight_per_Tire[nw - 1] * g;
//     }
//     return fmax;
// }

// void ChSalaaniTire::SetSalaaniPassengerCarParams(double tire_width,
//                                                  double aspect_ratio,
//                                                  double rim_diameter,
//                                                  double max_load) {
//     // Set basic tire geometry
//     m_width = tire_width;
//     double section_height = tire_width * aspect_ratio;
//     m_unloaded_radius = section_height + rim_diameter / 2.0;
//     m_rim_radius = rim_diameter / 2.0;

//     // Set reference load
//     m_salaani_coeff.FZ0 = max_load;
    
//     // Set friction coefficients
//     m_salaani_coeff.MUNOM = 0.9;
//     m_salaani_coeff.MUNTEST = 0.9;
    
//     // Lateral stiffness parameters (typical values for passenger car)
//     m_salaani_coeff.Cam = 180000.0;     // Maximum lateral stiffness [N/rad]
//     m_salaani_coeff.FZCam = max_load;   // Reference load for lateral stiffness
//     m_salaani_coeff.C1 = -0.3;          // Lateral stiffness coefficient
//     m_salaani_coeff.C2 = -0.1;          // Lateral stiffness coefficient
    
//     // Longitudinal stiffness parameters
//     m_salaani_coeff.Ckm = 150000.0;     // Maximum longitudinal stiffness [N]
//     m_salaani_coeff.FZCKM = max_load;   // Reference load for longitudinal stiffness
//     m_salaani_coeff.n_val = 0.1;        // Longitudinal stiffness exponent
    
//     // Peak friction coefficients
//     m_salaani_coeff.mu_p0_long = 1.2;   // Peak longitudinal friction
//     m_salaani_coeff.mu_p0_lat = 1.1;    // Peak lateral friction
//     m_salaani_coeff.eta0_long = -0.05;  // Longitudinal friction load sensitivity
//     m_salaani_coeff.eta1_long = 0.0;    // Longitudinal friction load sensitivity
//     m_salaani_coeff.eta2_lat = -0.1;    // Lateral friction load sensitivity
//     m_salaani_coeff.eta1_lat = 0.0;     // Lateral friction load sensitivity
    
//     // Friction decay parameters
//     m_salaani_coeff.d1_long = 0.0;      // Longitudinal friction decay
//     m_salaani_coeff.d2_long = 0.0;
//     m_salaani_coeff.d3_long = 0.3;
//     m_salaani_coeff.d1_lat = 0.0;       // Lateral friction decay
//     m_salaani_coeff.d2_lat = 0.0;
//     m_salaani_coeff.d3_lat = 0.2;
    
//     // Sliding friction scaling
//     m_salaani_coeff.epsilon_sy = 0.9;   // Lateral sliding friction scaling
//     m_salaani_coeff.epsilon_xx = 0.85;  // Longitudinal sliding friction scaling
    
//     // Pneumatic trail and overturning moment
//     m_salaani_coeff.tz1 = -1e-8;        // Pneumatic trail coefficient [1/N]
//     m_salaani_coeff.tz2 = 0.05;         // Pneumatic trail coefficient [m]
//     m_salaani_coeff.tx1 = -5e-9;        // Overturning moment arm coefficient
//     m_salaani_coeff.tx2 = 0.0;
//     m_salaani_coeff.tx3 = 0.01;         // Base overturning moment arm [m]
    
//     // Camber effects
//     m_salaani_coeff.Cr1 = 800.0;        // Camber force coefficient [N/rad]
//     m_salaani_coeff.Cr2 = 0.0;          // Camber force coefficient [N/rad/N]
    
//     // Aligning torque coefficients
//     m_salaani_coeff.m0 = 1.0;           // Base aligning torque coefficient
//     m_salaani_coeff.m1 = 10.0;          // Aligning torque load sensitivity
//     m_salaani_coeff.epsilon_x = 0.1;    // Aligning torque scaling factor
    
//     // Plysteer
//     m_salaani_coeff.plysteer = 0.0;     // No plysteer for passenger car
    
//     // Set vertical stiffness and damping
//     double deflection_at_load = 0.12 * section_height;  // 12% of section height
//     double vertical_stiffness = max_load / deflection_at_load;
//     SetVerticalStiffness(vertical_stiffness);
    
//     double damping_ratio = 0.3;  // Critical damping ratio
//     m_salaani_coeff.dz = 2.0 * damping_ratio * sqrt(vertical_stiffness * GetTireMass());
    
//     // Rolling resistance
//     m_rolling_resistance = 0.01;
// }

// void ChSalaaniTire::SetSalaaniTruckParams(double tire_width,
//                                          double aspect_ratio,
//                                          double rim_diameter,
//                                          double max_load) {
//     // Set basic tire geometry
//     m_width = tire_width;
//     double section_height = tire_width * aspect_ratio;
//     m_unloaded_radius = section_height + rim_diameter / 2.0;
//     m_rim_radius = rim_diameter / 2.0;

//     // Set reference load
//     m_salaani_coeff.FZ0 = max_load;
    
//     // Set friction coefficients
//     m_salaani_coeff.MUNOM = 0.8;
//     m_salaani_coeff.MUNTEST = 0.8;
    
//     // Lateral stiffness parameters (typical values for truck tire)
//     m_salaani_coeff.Cam = 250000.0;     // Maximum lateral stiffness [N/rad]
//     m_salaani_coeff.FZCam = max_load;   // Reference load for lateral stiffness
//     m_salaani_coeff.C1 = -0.25;         // Lateral stiffness coefficient
//     m_salaani_coeff.C2 = -0.08;         // Lateral stiffness coefficient
    
//     // Longitudinal stiffness parameters
//     m_salaani_coeff.Ckm = 200000.0;     // Maximum longitudinal stiffness [N]
//     m_salaani_coeff.FZCKM = max_load;   // Reference load for longitudinal stiffness
//     m_salaani_coeff.n_val = 0.15;       // Longitudinal stiffness exponent
    
//     // Peak friction coefficients
//     m_salaani_coeff.mu_p0_long = 1.0;   // Peak longitudinal friction
//     m_salaani_coeff.mu_p0_lat = 0.9;    // Peak lateral friction
//     m_salaani_coeff.eta0_long = -0.08;  // Longitudinal friction load sensitivity
//     m_salaani_coeff.eta1_long = 0.0;    // Longitudinal friction load sensitivity
//     m_salaani_coeff.eta2_lat = -0.12;   // Lateral friction load sensitivity
//     m_salaani_coeff.eta1_lat = 0.0;     // Lateral friction load sensitivity
    
//     // Friction decay parameters
//     m_salaani_coeff.d1_long = 0.0;      // Longitudinal friction decay
//     m_salaani_coeff.d2_long = 0.0;
//     m_salaani_coeff.d3_long = 0.4;
//     m_salaani_coeff.d1_lat = 0.0;       // Lateral friction decay
//     m_salaani_coeff.d2_lat = 0.0;
//     m_salaani_coeff.d3_lat = 0.3;
    
//     // Sliding friction scaling
//     m_salaani_coeff.epsilon_sy = 0.85;  // Lateral sliding friction scaling
//     m_salaani_coeff.epsilon_xx = 0.8;   // Longitudinal sliding friction scaling
    
//     // Pneumatic trail and overturning moment
//     m_salaani_coeff.tz1 = -8e-9;        // Pneumatic trail coefficient [1/N]
//     m_salaani_coeff.tz2 = 0.08;         // Pneumatic trail coefficient [m]
//     m_salaani_coeff.tx1 = -3e-9;        // Overturning moment arm coefficient
//     m_salaani_coeff.tx2 = 0.0;
//     m_salaani_coeff.tx3 = 0.015;        // Base overturning moment arm [m]
    
//     // Camber effects
//     m_salaani_coeff.Cr1 = 600.0;        // Camber force coefficient [N/rad]
//     m_salaani_coeff.Cr2 = 0.0;          // Camber force coefficient [N/rad/N]
    
//     // Aligning torque coefficients
//     m_salaani_coeff.m0 = 1.2;           // Base aligning torque coefficient
//     m_salaani_coeff.m1 = 12.0;          // Aligning torque load sensitivity
//     m_salaani_coeff.epsilon_x = 0.08;   // Aligning torque scaling factor
    
//     // Plysteer
//     m_salaani_coeff.plysteer = 0.0;     // No plysteer for truck tire
    
//     // Set vertical stiffness and damping
//     double deflection_at_load = 0.16 * section_height;  // 16% of section height
//     double vertical_stiffness = max_load / deflection_at_load;
//     SetVerticalStiffness(vertical_stiffness);
    
//     double damping_ratio = 0.5;  // Critical damping ratio
//     m_salaani_coeff.dz = 2.0 * damping_ratio * sqrt(vertical_stiffness * GetTireMass());
    
//     // Rolling resistance
//     m_rolling_resistance = 0.015;
// }

// Parameter consistency check
// bool ChSalaaniTire::CheckParameters() {
//     // Reference load set?
//     if (m_salaani_coeff.FZ0 <= 0.0) {
//         std::cerr << "SalaaniCheckParameters(): Incorrect tire reference load FZ0!" << std::endl;
//         return false;
//     }

//     // Vertical stiffness parameters
//     if (m_d1 <= 0.0) {
//         std::cerr << "SalaaniCheckParameters(): Incorrect tire vertical stiffness!" << std::endl;
//         return false;
//     }

//     // Friction coefficient
//     if (m_salaani_coeff.MUNOM <= 0.0) {
//         std::cerr << "SalaaniCheckParameters(): Incorrect coefficient of friction!" << std::endl;
//         return false;
//     }

//     // Vertical damping
//     if (m_vert_damping <= 0.0) {
//         std::cerr << "SalaaniCheckParameters(): Incorrect tire vertical damping!" << std::endl;
//         return false;
//     }

//     // Stiffness parameters
//     if (m_salaani_coeff.Cam <= 0.0 || m_salaani_coeff.Ckm <= 0.0) {
//         std::cerr << "SalaaniCheckParameters(): Incorrect tire stiffness parameters!" << std::endl;
//         return false;
//     }

//     return true;
// }

}  // end namespace vehicle
}  // end namespace chrono