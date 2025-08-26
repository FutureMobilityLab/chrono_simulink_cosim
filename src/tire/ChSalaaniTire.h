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

  /// Get visualization width.
  virtual double GetVisualizationWidth() const override { return m_width; }

  /// Get the tire slip angle computed internally by the Salaani model (in radians).
  double GetSlipAngle_internal() const { return m_states.alpha; }

  /// Get the tire longitudinal slip computed internally by the Salaani model.
  double GetLongitudinalSlip_internal() const { return m_states.slip_ratio; }

  double GetTireOmega() { return m_states.omega; }

  /// Get maximum tire load from Load Index (LI) in N [0:279].
  // static double GetTireMaxLoad(unsigned int li);

  /// Set vertical tire stiffness as linear function by coefficient [N/m].
  void SetVerticalStiffness(double Cz) {
    m_d1 = Cz;
    m_d2 = 0;
  }

  // /// Set vertical tire stiffness as nonlinear function by calculation from tire test data.
  // void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

  /// Set the tire reference coefficient of friction.
  void SetFrictionCoefficient(double coef) { m_salaani_coeff.MUNOM = coef; }

  /// Set rolling resistance coefficients (default: 0.01).
  void SetRollingResistanceCoefficient(double coef) { m_rolling_resistance = coef; }

  /// Get the tire deflection.
  virtual double GetDeflection() const override { return m_data.depth; }

  // /// Simple parameter consistency test.
  // bool CheckParameters();

 protected:
  /// Set the parameters in the Salaani model.
  virtual void SetSalaaniParams() = 0;

  /// Return the vertical tire stiffness contribution to the normal force.
  virtual double GetNormalStiffnessForce(double depth) const override final;

  /// Return the vertical tire damping contribution to the normal force.
  virtual double GetNormalDampingForce(double depth, double velocity) const override final;

  // Startup transition is used to scale Fx, Fy, and the moments in the 
  // beginning of simulation. Set |m_use_startup_transition| to activate it.
  // Set |m_begin_start_transition| to 
  bool m_use_startup_transition;
  double m_time;
  double m_begin_start_transition;
  double m_end_start_transition;

  // A constant that is added to the denominator of the longitudinal slip 
  // ratio calculation. This is used to avoid divide by zero.
  double m_vnum; 

  // Basic tire geometry parameters
  double m_unloaded_radius;     ///< reference tire radius
  double m_width;               ///< tire width
  double m_aspect_ratio;        ///< aspect ratio of the tire
  double m_rim_radius;          ///< tire rim radius
  double m_bottom_radius;       ///< radius where tire bottoming begins
  double m_bottom_stiffness;    ///< stiffness of the tire/bottom contact
  double m_rolling_resistance;  ///< actual rolling friction coeff

  double m_max_load;            ///< maximum load for the tire (determines 
                                /// force required to achieve 12% deflection)
  double m_d1;  ///< polynomial coefficient for stiffness interpolation, linear
  double m_d2;  ///< polynomial coefficient for stiffness interpolation, quadratic
  double m_vert_damping;  ///< vertical damping coefficient

  // Parameters used to smoothly blend between the Dahl model and the Salaani model
  // with a SineStep function (ChFunctionSineStep).
  // double m_vcoulomb;
  double m_frblend_begin;
  double m_frblend_end;

  VehicleSide m_measured_side;

  struct DahlCoeff {
    double sigma0{100000.0};  ///< bristle stiffness for Dahl friction model
    double sigma1{5000.0};    ///< bristle damping for Dahl friction model
  };

  struct SalaaniCoeff {
    // Tire lateral stiffness parameters (Equation 26)
    double C1, C2;        // Lateral stiffness coefficients (CA1, CA2 in MATLAB)
    double Cam;           // Maximum lateral stiffness (CAm in MATLAB)
    double FZCam;         // Reference load for lateral stiffness (CAFzm in MATLAB)
    
    // Tire longitudinal stiffness parameters (Equation 27)
    double Ckm;           // Initial longitudinal stiffness (CSO in MATLAB)
    double FZCKM;         // Reference load for longitudinal stiffness (FzxO in MATLAB)
    double n_val;         // Longitudinal stiffness exponent (Eta in MATLAB)
    
    // Lateral peak coefficient of friction (Equation 22)
    double eta1_lat;      // Lateral friction coefficient 1 (Muy1 in MATLAB)
    double eta2_lat;      // Lateral friction coefficient 2 (Muy2 in MATLAB)
    double mu_p0_lat;     // Lateral peak friction at reference load (MuyO in MATLAB)
    
    // Longitudinal peak coefficient of friction (Equation 22)
    double eta0_long;     // Longitudinal friction coefficient 0 (Mux2 in MATLAB)
    double eta1_long;     // Longitudinal friction coefficient 1 (Mux1 in MATLAB)
    double mu_p0_long;    // Longitudinal peak friction at reference load (MuxO in MATLAB)
      
    double FZ0;           // Reference load for friction calculations (FzO in MATLAB)
    
    // Lateral decay of friction (Equation 23)
    double d1_lat;        // Lateral decay coefficient 1 (KMUy1 in MATLAB)
    double d2_lat;        // Lateral decay coefficient 2 (KMUy2 in MATLAB)
    double d3_lat;        // Lateral decay coefficient 3 (KMUy3 in MATLAB)
    double epsilon_sy;    // Lateral sliding coefficient (Lsy in MATLAB)
    
    // Longitudinal decay of friction (Equation 23)
    double d1_long;       // Longitudinal decay coefficient 1 (KMUx1 in MATLAB)
    double d2_long;       // Longitudinal decay coefficient 2 (KMUx2 in MATLAB)
    double d3_long;       // Longitudinal decay coefficient 3 (KMUx3 in MATLAB)
    double epsilon_xx;    // Longitudinal sliding coefficient (Lsx in MATLAB)
    
    // Aligning moment pneumatic trail (Equation 29)
    double tz1, tz2;      // Pneumatic trail coefficients
    
    // Aligning moment constants (Equation 20)
    double epsilon_x;     // Sliding force eccentricity (Epsx in MATLAB)
    double m1, m0;        // Aligning moment constants
      
    // Overturning moment arm (Equation 30)
    double tx1, tx2, tx3; // Overturning moment arm coefficients
    
    // Inclination angle lateral force stiffness (Equation 31)
    double Cr1, Cr2;      // Camber stiffness coefficients (GAMMA1, GAMMA2 in MATLAB)
    
    // Additional parameters
    double Rt;            // Contact patch length
    double Cz_contact;    // Contact patch stiffness
    double beta;          // Tire relaxation length
    double plysteer;      // Plysteer offset
    
    // Friction coefficients (assumed nominal = test conditions)
    double MUNOM = 1.0;   // Nominal friction coefficient
    double MUNTEST = 1.0; // Test friction coefficient
  };

  SalaaniCoeff m_salaani_coeff;
  DahlCoeff m_dahl_coeff;

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

  // Tire states for the dynamic Dahl model at low speeds.
  struct TireStates {
    // States or intermediate variables used in the Dahl model.
    double muscale;              // Scaling factor for tire/road friction
    double vsx;                  // Longitudinal slip velocity
    double vsy;                  // Lateral slip velocity = Lateral velocity
    double brx{0};               // Bristle deformation x (for Dahl model)
    double bry{0};               // Bristle deformation y (for Dahl model)
    // Kinematic states or intermediate variables passed to the Salaani 
    // tire equations.
    double gamma;                // Camber angle [rad]
    double omega;                // Wheel angular velocity about its spin axis
    double R_eff;                // Effective rolling radius (used to calculate vta)
    double vta;                  // Absolute transport velocity (strictly positive)
    double slip_ratio;           // Longitudinal slip ratio
    double alpha;                // Slip angle [rad]
    // Logging of the |disc_normal| passed to DiscTerrainCollision().
    ChVector3d disc_normal;      // Disc normal vector
  };

  TireStates m_states;

  static void printTireStates(const TireStates& ts) {
    std::cout << "muscale: " << ts.muscale << "\n"
              << "vsx: " << ts.vsx << "\n"
              << "vsy: " << ts.vsy << "\n"
              << "brx: " << ts.brx << "\n"
              << "bry: " << ts.bry << "\n"
              << "gamma: " << ts.gamma << "\n"
              << "omega: " << ts.omega << "\n"
              << "R_eff: " << ts.R_eff << "\n"
              << "vta: " << ts.vta << "\n"
              << "slip_ratio: " << ts.slip_ratio << "\n"
              << "alpha: " << ts.alpha << "\n"
              << "disc_normal: [" << ts.disc_normal.x() << ", " 
              << ts.disc_normal.y() << ", " << ts.disc_normal.z() << "]\n";
  }

  // Convenience method to print this object's tire states
  void printMyTireStates() const {
    printTireStates(this->m_states);
  }
};

}  // end namespace vehicle
}  // end namespace chrono

#endif