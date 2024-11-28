// A linear force-based rack and pinion system. There is no pinion body, but to
// approximate it the torque applied is scaled by the pinion radius that is then
// applied to the rack as a linear force.

#ifndef CH_RACKPINIONFORCE_H
#define CH_RACKPINIONFORCE_H

// #include "chrono_vehicle/ChVehicleComponent.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono
{
  namespace vehicle
  {

    /// The steering subsystem is modeled with respect to a right-handed frame
    /// with with X pointing towards the front, Y to the left, and Z up (ISO
    /// standard). The steering link translates along the Y axis. We do not
    /// explicitly model the pinion but instead convert the pinion torque to a
    /// linear force by scaling it by the pinion radius.
    class ChRackPinionForce : public ChSteering
    {
    public:
      /// Construct a rack-pinion force steering mechanism with given base name.
      ChRackPinionForce(const std::string &name);

      virtual ~ChRackPinionForce();

      /// Get the name of the vehicle subsystem template.
      virtual std::string GetTemplateName() const override { return "RackPinion"; }

      /// @brief Initialize this steering subsystem.
      /// @param chassis  The chassis to which the steering is attached.
      /// @param location The location of the steering relative to the chassis.
      /// @param rotation The rotation of the steering relative to the chassis.
      virtual void Initialize(std::shared_ptr<ChChassis> chassis,
                              const ChVector<> &location,
                              const ChQuaternion<> &rotation) override;

      /// Add visualization assets for the steering subsystem.
      /// This default implementation uses primitives.
      virtual void AddVisualizationAssets(VisualizationType vis) override;

      /// Remove visualization assets for the steering subsystem.
      virtual void RemoveVisualizationAssets() override;

      /// @brief Apply a force to the pinion.
      /// @param force The force to apply [N].
      void ApplyForce(double force);

      /// Update the state of this steering subsystem at the current time.
      /// The steering subsystem is provided the current steering driver input (a value between -1 and +1).  Positive
      /// steering input indicates steering to the left. This function is called during the vehicle update.
      virtual void Synchronize(double time,                      ///< [in] current time
                               const DriverInputs &driver_inputs ///< [in] current driver inputs
                               ) override;

      /// Log current constraint violations.
      virtual void LogConstraintViolations() override;

    protected:
      virtual void InitializeInertiaProperties() override;
      virtual void UpdateInertiaProperties() override;

      /// Return the mass of the steering link.
      virtual double GetSteeringLinkMass() const = 0;

      /// Return the moments of inertia of the steering link.
      virtual ChVector<> GetSteeringLinkInertia() const = 0;

      /// Return the steering link COM offset in Y direction (positive to the left).
      virtual double GetSteeringLinkCOM() const = 0;

      /// Return the radius of the steering link (visualization only).
      virtual double GetSteeringLinkRadius() const = 0;

      /// Return the length of the steering link (visualization only).
      virtual double GetSteeringLinkLength() const = 0;

      /// Return the radius of the pinion.
      virtual double GetPinionRadius() const = 0;

      /// Return the maximum rotation angle of the pinion (in either direction).
      virtual double GetMaxAngle() const = 0;

      virtual void ExportComponentList(rapidjson::Document &jsonDocument) const override;

      virtual void Output(ChVehicleOutput &database) const override;

      std::shared_ptr<ChLinkLockPrismatic> m_prismatic; ///< handle to the prismatic joint chassis-link
      std::shared_ptr<ChLinkMotorLinearForce> m_motor;  ///< handle to the linear actuator on steering link
      std::shared_ptr<ChLinkTSDA> m_springDamper;       ///< handle to the spring-damper on rack
    };

  } // namespace vehicle
} // namespace chrono

#endif