#include "src/suspension_test_rig/SuspensionTestRig.h"

#include <chrono/core/ChTypes.h>
#include <chrono/utils/ChUtils.h>  // For ChClamp
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h>

namespace chrono::vehicle
{
  CosimSuspensionTestRig::CosimSuspensionTestRig() : ChSuspensionTestRigDriver() {}

  CosimSuspensionTestRig::~CosimSuspensionTestRig() {}

  /// Set the value for the driver left post displacement input.
  void CosimSuspensionTestRig::SetDisplacementLeft(int axle, double val, double min_val, double max_val)
  {
    ChSuspensionTestRigDriver::SetDisplacementLeft(axle, val, min_val, max_val);
  }

  /// Set the value for the driver right post displacement input.
  void CosimSuspensionTestRig::SetDisplacementRight(int axle, double val, double min_val, double max_val)
  {
    ChSuspensionTestRigDriver::SetDisplacementRight(axle, val, min_val, max_val);
  }

  /// Set the value for the driver steering input.
  void CosimSuspensionTestRig::SetSteering(double val, double min_val, double max_val)
  {
    ChSuspensionTestRigDriver::SetSteering(val, min_val, max_val);
  }

  double CosimSuspensionTestRig::GetDisplacementLeft(int axle) const
  {
    return ChSuspensionTestRigDriver::GetDisplacementLeft()[axle];
  }

  double CosimSuspensionTestRig::GetDisplacementRight(int axle) const
  {
    return ChSuspensionTestRigDriver::GetDisplacementRight()[axle];
  }

  double CosimSuspensionTestRig::GetSteering() const
  {
    return ChSuspensionTestRigDriver::GetSteering();
  }

  void CosimSuspensionTestRig::Synchronize(double time)
  {
    ChSuspensionTestRigDriver::Synchronize(time);
  }

  std::string CosimSuspensionTestRig::GetInfoMessage() const
  {
    return "Cosimulation driver inputs";
  }

  void CosimSuspensionTestRig::Initialize(int naxles)
  {
    ChSuspensionTestRigDriver::Initialize(naxles);
  }

  // Function used to create a suspension test rig from a specific vehicle. The
  // configuration is done by setting members of the Generic_STR_Setup class.
  std::shared_ptr<ChSuspensionTestRig> CreateFromVehicleModel(RigMode rig_mode, STR_Setup *setup)
  {
    std::cout << "Using vehicle specification file: " << setup->VehicleJSON() << std::endl;

    // Create the vehicle
    auto vehicle =
        chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(setup->VehicleJSON()), ChContactMethod::SMC);

    // Create the suspension test rig
    std::shared_ptr<ChSuspensionTestRig> rig;
    switch (rig_mode)
    {
    default:
    {
      throw std::runtime_error("Invalid rig mode.");
    }
    case RigMode::PLATFORM:
    {
      rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, setup->TestAxles(), setup->PostLimit());
      break;
    }
    case RigMode::PUSHROD:
    {
      rig = chrono_types::make_shared<ChSuspensionTestRigPushrod>(vehicle, setup->TestAxles(), setup->PostLimit());
      break;
    }
    }

    // Include additional subsystems in test
    for (auto is : setup->TestSteerings())
      rig->IncludeSteeringMechanism(is);
    for (auto is : setup->TestSubchassis())
      rig->IncludeSubchassis(is);

    rig->SetInitialRideHeight(setup->InitRideHeight());

    return rig;
  }

  // Function used to create a suspension test rig from a suspension test rig
  // configuration file (a json file).
  std::shared_ptr<ChSuspensionTestRig> CreateFromSpecFile(RigMode rig_mode, STR_Setup *setup)
  {
    std::cout << "Using STR specification file: " << setup->SuspensionRigJSON() << std::endl;

    switch (rig_mode)
    {
    default:
    case RigMode::PLATFORM:
      return chrono_types::make_shared<ChSuspensionTestRigPlatform>(
          vehicle::GetDataFile(setup->SuspensionRigJSON()));
    case RigMode::PUSHROD:
      return chrono_types::make_shared<ChSuspensionTestRigPushrod>(
          vehicle::GetDataFile(setup->SuspensionRigJSON()));
    }
  }
} // namespace chrono::vehicle