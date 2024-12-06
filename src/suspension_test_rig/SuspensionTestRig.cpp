#include "src/suspension_test_rig/SuspensionTestRig.h"

#include "chrono/core/ChTypes.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

namespace chrono::vehicle
{
  /// Set the value for the driver left post displacement input.
  void CosimSuspensionTestRig::SetDisplacementLeft(int axle, double val, double min_val, double max_val)
  {
    m_displLeft[axle] = ChClamp(val, min_val, max_val);
  }

  /// Set the value for the driver right post displacement input.
  void CosimSuspensionTestRig::SetDisplacementRight(int axle, double val, double min_val, double max_val)
  {
    m_displRight[axle] = ChClamp(val, min_val, max_val);
  }

  /// Set the value for the driver steering input.
  void CosimSuspensionTestRig::SetSteering(double val, double min_val, double max_val)
  {
    m_steering = ChClamp(val, min_val, max_val);
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
      auto rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, setup->TestAxles(), setup->PostLimit());
      break;
    }
    case RigMode::PUSHROD:
    {
      auto rig = chrono_types::make_shared<ChSuspensionTestRigPushrod>(vehicle, setup->TestAxles(), setup->PostLimit());
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
} // chrono::vehicle