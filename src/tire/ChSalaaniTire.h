// =============================================================================
// Authors: Trevor Vidano
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
//
// Features:
//  - Combined slip handling
//  - Low-speed Dahl friction model blending (similar to TMeasy)
//  - Camber angle effects
//  - Aligning torque and overturning moment calculations
// ===================================================================================

#ifndef CH_SALAANI_TIRE
#define CH_SALAANI_TIRE

#include <vector>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

#include "src/tire/salaani_model.h"

#include <memory>

namespace chrono {
namespace vehicle {

/// Salaani unified tire model.
class CH_VEHICLE_API ChSalaaniTire : public ChForceElementTire {
 public:
  ChSalaaniTire(const std::string& name);

  virtual ~ChSalaaniTire() {}

  /// Get the name of the vehicle subsystem template.
  virtual std::string GetTemplateName() const override { return "SalaaniTire"; }

  /// Get the tire radius.
  virtual double GetRadius() const override { return m_states.R_eff; }

  /// Get the width of the tire.
  virtual double GetWidth() const override { return m_width; }

  /// Get the tire deflection.
  virtual double GetDeflection() const override { return m_data.depth; }

  /// Get visualization width.
  virtual double GetVisualizationWidth() const override { return m_width; }

  /// Get the tire slip angle computed internally by the Salaani model (in radians).
  double GetSlipAngle_internal() const { return m_states.alpha; }

  /// Get the tire longitudinal slip computed internally by the Salaani model.
  double GetLongitudinalSlip_internal() const { return m_states.slip_ratio; }

  double GetTireOmega() { return m_states.omega; }

  // This is already performed by ChTire::CalculateKinematics, but is done in 
  // ChPac89Tire for some reason (likely legacy that has not been updated yet).
  // double GetCamberAngle_internal() const { return m_gamma; }

  /// From TMeasy: Get maximum tire load from Load Index (LI) in N [0:279].
  // static double GetTireMaxLoad(unsigned int li);

  /// From TMeasy: Set vertical tire stiffness as linear function by coefficient [N/m].
  // void SetVerticalStiffness(double Cz) {
  //   m_d1 = Cz;
  //   m_d2 = 0;
  // }

  /// From TMeasy: Set vertical tire stiffness as nonlinear function by calculation from tire test data.
  // void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

  /// Set the tire reference coefficient of friction.
  // void SetFrictionCoefficient(double coef) { MUNOM = coef; }

  /// From TMeasy: Set rolling resistance coefficients (default: 0.01).
  // void SetRollingResistanceCoefficient(double coef) { m_rolling_resistance = coef; }

  // /// Simple parameter consistency test.
  // bool CheckParameters();

 protected:
  // From TMeasy:
  struct DahlCoeff {
    double sigma0{100000.0};  ///< bristle stiffness for Dahl friction model
    double sigma1{5000.0};    ///< bristle damping for Dahl friction model
  };

  // Tire states for the dynamic Dahl model at low speeds.
  struct TireStates {
    // States or intermediate variables used in the Dahl model.
    double muscale;              // Scaling factor for tire/road friction
    double vx;                   // longitudinal speed
    double vsx;                  // Longitudinal slip velocity
    double vsy;                  // Lateral slip velocity = Lateral velocity
    double brx{0};               // Bristle deformation x (for Dahl model)
    double bry{0};               // Bristle deformation y (for Dahl model)
    // Kinematic states or intermediate variables passed to the Salaani 
    // tire equations.
    double gamma;                // Camber angle [rad]
    double omega;                // Wheel angular velocity about its spin axis
    double R_eff;                // Effective rolling radius (used to calculate vta)
    // double vta;                  // Absolute transport velocity (strictly positive)
    double slip_ratio;           // Longitudinal slip ratio
    double instant_slip_ratio;   // Instantaneous slip ratio
    double alpha;                // Slip angle [rad]
    double instant_alpha;        // Instantaneous slip angle [rad]
    // Logging of the |disc_normal| passed to DiscTerrainCollision().
    ChVector3d disc_normal;      // Disc normal vector
  };

  /// Set the parameters in the Salaani model.
  virtual void SetSalaaniParams() = 0;

  /// Initialize this tire by associating it to the specified wheel.
  virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

  /// Update the state of this tire system at the current time.
  virtual void Synchronize(double time,              ///< [in] current time
                            const ChTerrain& terrain  ///< [in] reference to the terrain system
                            ) override;

  /// Advance the state of this tire by the specified time step.
  virtual void Advance(double step) override;

  /// Calculate tire forces using the Salaani model
  void CalculateSalaaniForces(double& fx, double& fy, double& mz, double& mx, 
                              double alpha, double slip_ratio, double gamma, double fz);

  /// Calculate forces using Dahl friction model for low speeds
  /*
    The Dahl Friction Model elastic tread blocks representated by a single bristle. At tire stand still it acts
    like a spring which enables holding of a vehicle on a slope without creeping (hopefully). Damping terms
    have been added to calm down the oscillations of the pure spring.

    The time step h must be actually the same as for the vehicle system!

    This model is experimental and needs some testing.

    With bristle deformation z, Coulomb force fc, sliding velocity v and stiffness sigma we have this
    differential equation:
        dz/dt = v - sigma0*z*abs(v)/fc

    When z is known, the friction force F can be calulated to:
      F = sigma0 * z

    For practical use some damping is needed, that leads to:
      F = sigma0 * z + sigma1 * dz/dt

    Longitudinal and lateral forces are calculated separately and then combined. For stand still a friction
    circle is used.
  */
  void CombinedCoulombForces(double& fx, double& fy, double fz, double muscale);

  // Convenience method to print this object's tire states
  void printMyTireStates() const;

  // Basic tire geometry parameters
  double m_unloaded_radius;     ///< reference tire radius
  double m_width;               ///< tire width
  // double m_aspect_ratio;        ///< aspect ratio of the tire
  // double m_rim_radius;          ///< tire rim radius
  // double m_bottom_radius;       ///< radius where tire bottoming begins
  // double m_bottom_stiffness;    ///< stiffness of the tire/bottom contact
  double m_rolling_resistance;  ///< actual rolling friction coeff

  // double m_max_load;            ///< maximum load for the tire (determines 
                                /// force required to achieve 12% deflection)
  // double m_d1;  ///< polynomial coefficient for stiffness interpolation, linear
  // double m_d2;  ///< polynomial coefficient for stiffness interpolation, quadratic
  // double m_vert_damping;  ///< vertical damping coefficient

  // Parameters used to smoothly blend between the Dahl model and the Salaani model
  // with a SineStep function (ChFunctionSineStep).
  // double m_vcoulomb;
  // double m_frblend_begin;
  // double m_frblend_end;

  VehicleSide m_measured_side;

  DahlCoeff m_dahl_coeff;

  TireStates m_states;

  std::unique_ptr<tire::SalaaniTireModel> salaani_model;

  // Relaxation length.
  bool m_enable_lateral_relaxation = false;
  double m_lateral_relaxation_length;

  // Speed-based tire model blending.
  bool m_enable_speed_based_tire_model_blending = false;
  double m_frblend_begin = 0.0;
  double m_frblend_end = 0.0;
  // double MUNOM;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif