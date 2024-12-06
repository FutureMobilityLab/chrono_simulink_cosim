#ifndef SUSPENSION_TESTRIG_H
#define SUSPENSION_TESTRIG_H

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

namespace chrono::vehicle
{
  // =============================================================================
  // Class that enables direct access to the rig inputs.
  class CosimSuspensionTestRig : public ChDriverSTR
  {
  public:
    /// Set the value for the driver left post displacement input.
    void SetDisplacementLeft(int axle, double val, double min_val = -10, double max_val = 10);

    /// Set the value for the driver right post displacement input.
    void SetDisplacementRight(int axle, double val, double min_val = -10, double max_val = 10);

    /// Set the value for the driver steering input.
    void SetSteering(double val, double min_val = -10, double max_val = 10);
  };

  class STR_Setup
  {
  public:
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
} // chrono::vehicle

#endif