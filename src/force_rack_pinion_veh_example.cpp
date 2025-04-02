// =============================================================================
// Authors: Trevor Vidano
//
// Date: 11/26/2024
// =============================================================================
//
// This file creates a Ford Expedition 2003 with a rack and pinion subsystem
// that uses a linear force as an input. The throttle and brakes are left as
// zero so that the vehicle simply is created and a sinusoidal input to the rack
// and pinion moves the steering.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

// #include "chrono/solver/ChIterativeSolverLS.h"

// #include "chrono/utils/ChUtilsInputOutput.h"
// #include "chrono/utils/ChFilters.h"

// #include "chrono_vehicle/ChConfigVehicle.h"
// #include "chrono_vehicle/ChVehicleModelData.h"
// #include "chrono_vehicle/ChPowertrainAssembly.h"
// #include "chrono_vehicle/terrain/RigidTerrain.h"
// #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
// #include "chrono_vehicle/utils/ChUtilsJSON.h"
// #include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
// #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

// #include "chrono_thirdparty/filesystem/path.h"

// #include "chrono/utils/ChSocketCommunication.h"

#include "src/simulation_interface/SimulationInterface.h"
// #include "src/vehicle/WheeledVehicleForce.h"

// #include <filesystem>
// #include <iostream>

namespace chrono
{
  // =============================================================================
//   class Vehicle_Model {
//   public:
//     virtual std::string ModelName() const = 0;
//     virtual std::string VehicleJSON() const = 0;
//     virtual std::string TireJSON(unsigned int axle) const = 0;
//     virtual std::string EngineJSON() const = 0;
//     virtual std::string TransmissionJSON() const = 0;
//     virtual double CameraDistance() const = 0;
//     virtual ChContactMethod ContactMethod() const = 0;
// };

// class Sedan_Model : public Vehicle_Model {
//   public:
//     virtual std::string ModelName() const override { return "Sedan"; }
//     virtual std::string VehicleJSON() const override { return "sedan_force/vehicle/Sedan_Vehicle.json"; }
//     virtual std::string TireJSON(unsigned int axle) const override {
//         ////return "sedan_force/tire/Sedan_RigidTire.json";
//         return "sedan_force/tire/Sedan_TMeasyTire.json";
//         // return "sedan_force/tire/Sedan_Pac02Tire.json";
//     }
//     virtual std::string EngineJSON() const override {
//         ////return "sedan_force/powertrain/Sedan_EngineSimpleMap.json";
//         return "sedan_force/powertrain/Sedan_EngineShafts.json";
//     }
//     virtual std::string TransmissionJSON() const override {
//         ////return "sedan_force/powertrain/Sedan_AutomaticTransmissionSimpleMap.json";
//         return "sedan_force/powertrain/Sedan_ManualTransmissionShafts.json";
//     }
//     virtual double CameraDistance() const override { return 6.0; }
//     virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
// };
} // namespace chrono

// =============================================================================

int main(int argc, char *argv[])
{
  auto simulation_interface = simulation_interface::Simulation_Interface("sedan");
  auto vis = simulation_interface.get_vis();
  double input[simulation_interface::Input::LENGTH] = {0.0, 0.0, 0.0};
  double output[simulation_interface::Output::LENGTH] = {0.0, 0.0};
  while (vis->Run()) {
    simulation_interface.Step(input, output);
  }
  return 0;
}
