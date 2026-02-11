// =============================================================================
// Authors: Trevor Vidano
// =============================================================================
//
// Template for a tire model based on Salaani's extension of the STI tire model
// to low friction surfaces. This tire is developed in:
//  - Salaani, M. K., et al. (2006). Measurement and Modeling of Tire Forces on
//    a Low Coefficient Surface (SAE Technical Paper 2006-01-0559). SAE
//    International.
//
// Force and moment calculation is delegated to the STIREMOD kernel
// (stiremod_calculate). The Dahl bristle model is retained for the
// low-speed / stand-still regime and blended into the STIREMOD output.
//
// Unit conventions
// ----------------
//   Chrono internals : SI  (m, N, N·m, rad)
//   STIREMOD kernel  : Imperial (lbs, ft, in, psi, degrees)
//
// All unit conversions are localised to the Advance() method.
//   Fz    : N       → lbs    (multiply by LBS_PER_N)
//   Fx,Fy : lbs     → N      (multiply by N_PER_LB)
//   Mz    : ft·lbs  → N·m    (multiply by NM_PER_FTLB)
//
// Friction scaling
// ----------------
// STIREMOD models friction through peak-mu coefficients (B-parameters) and
// skid-number ratios. To honour the terrain friction reported by Chrono we
// scale the skid-number ratio by (m_mu / m_mu0) so that, on a surface with
// mu = m_mu0, the unmodified STIREMOD coefficients are used exactly.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/functions/ChFunctionSineStep.h"

#include "src/tire/ChStiremodTire.h"

namespace chrono {
namespace vehicle {

// ---------------------------------------------------------------------------
// Unit-conversion constants
// ---------------------------------------------------------------------------
static constexpr double LBS_PER_N   = 0.224808943;  // 1 N      → lbs
static constexpr double N_PER_LB    = 4.44822162;   // 1 lb     → N
static constexpr double NM_PER_FTLB = 1.35581795;   // 1 ft·lb  → N·m

// ---------------------------------------------------------------------------
ChStiremodTire::ChStiremodTire(const std::string& name)
    : ChForceElementTire(name),
      m_gamma(0),
      m_gamma_limit(3.0 * CH_DEG_TO_RAD),
      m_mu(0),
      m_mu0(0.8) {
    m_tireforce.force  = ChVector3d(0, 0, 0);
    m_tireforce.point  = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);
}

// ---------------------------------------------------------------------------
void ChStiremodTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetStiremodParams();

    // Build the lookup table for penetration depth as a function of
    // intersection area (used only with the ENVELOPE collision method).
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialise contact-patch state variables.
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.R_eff        = m_unloaded_radius;
}

// ---------------------------------------------------------------------------
void ChStiremodTire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame).
    ChMatrix33<> A(wheel_state.rot);
    ChVector3d disc_normal = A.GetAxisY();

    // Check disc-terrain contact.
    float mu;
    m_data.in_contact =
        DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                             m_width, m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu = mu;

    // Calculate tire kinematics.
    CalculateKinematics(wheel_state, m_data.frame);

    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C frame.
        ChVector3d vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Normal contact force. A negative value means the disc is separating;
        // treat as no contact.
        double Fn_mag =
            GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;
        }

        m_data.normal_force  = Fn_mag;
        m_states.R_eff       = m_unloaded_radius - m_data.depth;
        m_states.vx          = std::abs(m_data.vel.x());
        m_states.vsx         = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy         = -m_data.vel.y();  // modified-SAE sign convention
        m_states.omega       = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states when the tire leaves the ground.
        m_data.normal_force   = 0;
        m_states.R_eff        = m_unloaded_radius;
        m_states.cp_long_slip = 0;
        m_states.cp_side_slip = 0;
        m_states.vx           = 0;
        m_states.vsx          = 0;
        m_states.vsy          = 0;
        m_states.omega        = 0;
        m_states.brx          = 0;
        m_states.bry          = 0;
        m_states.disc_normal  = ChVector3d(0, 0, 0);
    }
}

// ---------------------------------------------------------------------------
void ChStiremodTire::Advance(double step) {
    // Zero tire forces.
    m_tireforce.force  = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);

    if (!m_data.in_contact)
        return;

    // ------------------------------------------------------------------
    // 1.  Compute kinematic slip quantities
    // ------------------------------------------------------------------

    // Longitudinal slip (kappa)
    if (m_states.vx != 0) {
        m_states.cp_long_slip = -m_states.vsx / m_states.vx;
    } else {
        m_states.cp_long_slip = 0;
    }

    // Side-slip angle (alpha) – in the modified-SAE frame used by STIREMOD
    if (m_states.omega != 0) {
        m_states.cp_side_slip = std::atan(
            m_states.vsy / std::abs(m_states.omega * (m_unloaded_radius - m_data.depth)));
    } else {
        m_states.cp_side_slip = 0;
    }

    // Clamp slip values to physically meaningful ranges.
    ChClampValue(m_states.cp_long_slip, -1.0, 1.0);
    ChClampValue(m_states.cp_side_slip, -CH_PI_2 + 0.001, CH_PI_2 - 0.001);

    // ------------------------------------------------------------------
    // 2.  Low-speed Dahl bristle forces (Fx0, Fy0) — used for blending
    // ------------------------------------------------------------------
    double Fx0 = 0, Fy0 = 0;
    double mu_scale = m_mu / m_mu0;
    CombinedCoulombForces(Fx0, Fy0, m_data.normal_force, mu_scale);

    // ------------------------------------------------------------------
    // 3.  Camber angle
    // ------------------------------------------------------------------
    m_gamma = ChClamp(CH_PI_2 - std::acos(m_states.disc_normal.z()),
                      -m_gamma_limit, m_gamma_limit);
    double gamma_deg = m_gamma * CH_RAD_TO_DEG;

    // ------------------------------------------------------------------
    // 4.  STIREMOD force kernel  (Imperial units)
    // ------------------------------------------------------------------
    // Convert Fz from N to lbs.
    double Fz_lbs = m_data.normal_force * LBS_PER_N;

    // STIREMOD uses the modified-SAE alpha convention: positive alpha
    // produces negative Fy (handled inside stiremod_calculate).
    double alpha_deg = m_states.cp_side_slip * CH_RAD_TO_DEG;
    double kappa     = m_states.cp_long_slip;

    // Scale terrain friction through the skid-number ratio.
    // When m_mu == m_mu0 the ratio is 1.0 and the nominal STIREMOD
    // coefficients apply exactly.
    StiremodParams scaled = m_stiParams;
    scaled.SN_o = m_stiParams.SN_t * mu_scale;

    auto sti = stiremod_calculate(scaled, Fz_lbs, alpha_deg, kappa, gamma_deg);

    // Convert forces from lbs → N and moment from ft·lbs → N·m.
    double Fx = sti.Fx * N_PER_LB;
    double Fy = sti.Fy * N_PER_LB;
    double Mz = sti.Mz * NM_PER_FTLB;

    // ------------------------------------------------------------------
    // 5.  Blend low-speed Dahl forces into STIREMOD output
    // ------------------------------------------------------------------
    constexpr double frblend_begin = 1.0;  // m/s
    constexpr double frblend_end   = 3.0;  // m/s
    double frblend = ChFunctionSineStep::Eval(m_data.vel.x(), frblend_begin, 0.0, frblend_end, 1.0);
    Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
    Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

    // ------------------------------------------------------------------
    // 6.  Overturning moment (Mx)
    // ------------------------------------------------------------------
    double Mx = 0;
    {
        double deflection = Fy / m_lateral_stiffness;
        Mx = -(m_data.normal_force) * deflection;
        Mz = Mz + Fx * deflection;
    }

    // ------------------------------------------------------------------
    // 7.  Rolling resistance moment (My)
    // ------------------------------------------------------------------
    double My = 0;
    {
        double Lrad      = m_unloaded_radius - m_data.depth;
        const double vx_min = 0.125;
        const double vx_max = 0.5;
        double myStartUp = ChFunctionSineStep::Eval(std::abs(m_states.vx), vx_min, 0.0, vx_max, 1.0);
        My = myStartUp * m_rolling_resistance * m_data.normal_force * Lrad * ChSignum(m_states.omega);
    }

    // ------------------------------------------------------------------
    // 8.  Assemble force/moment vector
    //     Convert from modified-SAE to ISO at the contact patch:
    //       Fy_ISO = -Fy_SAE
    //       My_ISO = -My_SAE
    //       Mz_ISO = -Mz_SAE
    // ------------------------------------------------------------------
    m_tireforce.force  = ChVector3d(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector3d(Mx, -My, -Mz);
}

// ---------------------------------------------------------------------------
// Dahl bristle model (low-speed / stand-still friction)
// ---------------------------------------------------------------------------
void ChStiremodTire::CombinedCoulombForces(double& fx, double& fy,
                                            double fz, double muscale) {
    /*
     * Single-bristle Dahl friction model. At stand-still it behaves like a
     * spring (preventing slow creep on slopes). A damping term damps
     * oscillations. Longitudinal and lateral forces are computed independently
     * then combined through a friction circle.
     *
     *   dz/dt = v - (sigma0 * |v| / fc) * z          (bristle ODE)
     *   F     = sigma0 * z + sigma1 * dz/dt           (force)
     *
     * Integration uses the implicit Euler method for numerical stability.
     */

    double fc = fz * muscale;      // Coulomb friction limit (N)
    double h  = this->m_stepsize;  // integration step (s)

    ChVector2d F;

    // Longitudinal
    double brx_dot = m_states.vsx
                   - m_sigma0 * m_states.brx * std::fabs(m_states.vsx) / fc;
    F.x() = -(m_sigma0 * m_states.brx + m_sigma1 * brx_dot);

    // Lateral
    double bry_dot = m_states.vsy
                   - m_sigma0 * m_states.bry * std::fabs(m_states.vsy) / fc;
    F.y() = -(m_sigma0 * m_states.bry + m_sigma1 * bry_dot);

    // Implicit Euler state update
    m_states.brx = (fc * m_states.brx + fc * h * m_states.vsx)
                 / (fc + h * m_sigma0 * std::fabs(m_states.vsx));
    m_states.bry = (fc * m_states.bry + fc * h * m_states.vsy)
                 / (fc + h * m_sigma0 * std::fabs(m_states.vsy));

    // Friction circle saturation
    if (F.Length() > fz * muscale) {
        F.Normalize();
        F *= fz * muscale;
    }

    fx = F.x();
    fy = F.y();
}

}  // end namespace vehicle
}  // end namespace chrono