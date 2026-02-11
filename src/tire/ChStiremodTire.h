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
// The force calculation kernel uses the STI (Systems Technologies Inc.) tire
// model as implemented in the stiremod library (stiremod.h / stiremod.cpp).
// All STIREMOD parameters are in Imperial units (lbs, inches, psi). Unit
// conversion to/from Chrono's SI system is handled internally.
//
// The low-speed / stand-still regime uses the Dahl bristle friction model,
// blended smoothly into the STIREMOD forces above ~3 m/s
// (frblend_begin / frblend_end).
//
// =============================================================================

#ifndef CH_STIREMODTIRE_H
#define CH_STIREMODTIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/ChTerrain.h"

#include "src/tire/stiremod.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// STI tire model (STIREMOD force kernel) base template.
///
/// Derived classes must implement SetStiremodParams() to populate
/// m_unloaded_radius, m_width, m_rolling_resistance, m_lateral_stiffness,
/// and m_stiParams (StiremodParams).
class CH_VEHICLE_API ChStiremodTire : public ChForceElementTire {
  public:
    ChStiremodTire(const std::string& name);

    virtual ~ChStiremodTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "StiremodTire"; }

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Set the limit for camber angle (in degrees). Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit * CH_DEG_TO_RAD; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_width; }

    /// Get the tire deflection.
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const override { return m_width; }

    /// Get the slip angle used internally by the STIREMOD kernel.
    /// Expressed in radians; sign convention follows the modified SAE frame
    /// used internally (opposite to ChTire::GetSlipAngle).
    double GetSlipAngle_internal() const { return m_states.cp_side_slip; }

    /// Get the longitudinal slip used internally by the STIREMOD kernel.
    double GetLongitudinalSlip_internal() const { return m_states.cp_long_slip; }

    /// Get the camber angle used internally by the STIREMOD kernel.
    /// Expressed in radians; sign convention matches ChTire::GetCamberAngle.
    double GetCamberAngle_internal() const { return m_gamma; }

  protected:
    /// Populate m_unloaded_radius, m_width, m_rolling_resistance,
    /// m_lateral_stiffness, and m_stiParams in the derived class.
    virtual void SetStiremodParams() = 0;

    // -------------------------------------------------------------------------
    // Parameters set by the derived class via SetStiremodParams()
    // -------------------------------------------------------------------------

    double m_unloaded_radius;     ///< Unloaded tire radius (m)
    double m_width;               ///< Tire section width (m)
    double m_rolling_resistance;  ///< Rolling resistance coefficient
    double m_lateral_stiffness;   ///< Lateral stiffness for overturning moment (N/m)
    VehicleSide m_measured_side;

    /// STIREMOD model parameters (all in Imperial units as required by stiremod.h)
    StiremodParams m_stiParams;

    /// Bristle stiffness for the Dahl low-speed friction model (N/m, SI)
    double m_sigma0{100000.0};
    /// Bristle damping for the Dahl low-speed friction model (N·s/m, SI)
    double m_sigma1{5000.0};

    // -------------------------------------------------------------------------
    // Internal state
    // -------------------------------------------------------------------------

    double m_gamma;        ///< Camber angle (rad)
    double m_gamma_limit;  ///< Camber angle clamp limit (rad)

    double m_mu;   ///< Road friction coefficient (from terrain query)
    double m_mu0;  ///< Reference friction coefficient for the STIREMOD params

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time, const ChTerrain& terrain) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    /// Low-speed Dahl bristle friction model.
    /// Computes combined Coulomb forces fx, fy (SI, Newtons) given normal
    /// force fz (SI, Newtons) and friction scaling factor muscale.
    void CombinedCoulombForces(double& fx, double& fy, double fz, double muscale);

    struct TireStates {
        double cp_long_slip;     ///< Longitudinal slip (kappa, dimensionless)
        double cp_side_slip;     ///< Side slip angle (alpha, rad, modified-SAE sign)
        double vx;               ///< Longitudinal speed at contact (m/s)
        double vsx;              ///< Longitudinal slip velocity (m/s)
        double vsy;              ///< Lateral slip velocity (m/s)
        double omega;            ///< Wheel spin rate (rad/s)
        double R_eff;            ///< Effective rolling radius (m)
        double brx{0};           ///< Dahl bristle deformation, longitudinal (m)
        double bry{0};           ///< Dahl bristle deformation, lateral (m)
        ChVector3d disc_normal;  ///< Wheel disc normal in global frame (debug)
    };

    TireStates m_states;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif