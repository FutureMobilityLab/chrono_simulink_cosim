#ifndef SUSPENSION_TESTRIG_H
#define SUSPENSION_TESTRIG_H

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono::vehicle
{
  // =============================================================================
  // Class that enables direct access to the rig inputs.
  class CosimSuspensionTestRig : public ChSuspensionTestRigDriver
  {
  public:
    CosimSuspensionTestRig();
    ~CosimSuspensionTestRig();

    /// Set the value for the driver left post displacement input.
    void SetDisplacementLeft(int axle, double val, double min_val = -10, double max_val = 10);

    /// Set the value for the driver right post displacement input.
    void SetDisplacementRight(int axle, double val, double min_val = -10, double max_val = 10);

    /// Set the value for the driver steering input.
    void SetSteering(double val, double min_val = -10, double max_val = 10);

    /// Get the driver left post displacement input.
    double GetDisplacementLeft(int axle) const;

    /// Get the driver right post displacement input.
    double GetDisplacementRight(int axle) const;

    /// Get the driver steering input.
    double GetSteering() const;

    /// Update the driver inputs based on current time.
    virtual void Synchronize(double time) override;

    /// Get string message about the driver's state
    virtual std::string GetInfoMessage() const override;

    /// Initialize this driver system.
    virtual void Initialize(int naxles) override;
  };

  class STR_Setup
  {
  public:
    virtual ~STR_Setup() {}
    virtual std::string SuspensionRigJSON() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string DataDriverFile() const = 0;
    virtual std::vector<int> TestAxles() const = 0;
    virtual std::vector<int> TestSubchassis() const = 0;
    virtual std::vector<int> TestSteerings() const = 0;
    virtual double InitRideHeight() const = 0;
    virtual double PostLimit() const = 0;
    virtual double CameraDistance() const = 0;
  };

  // STR rig type
  enum class RigMode
  {
    PLATFORM,
    PUSHROD
  };

  std::shared_ptr<ChSuspensionTestRig> CreateFromVehicleModel(RigMode rig_mode, STR_Setup *setup);
  std::shared_ptr<ChSuspensionTestRig> CreateFromSpecFile(RigMode rig_mode, STR_Setup *setup);
} // namespace chrono::vehicle

#endif