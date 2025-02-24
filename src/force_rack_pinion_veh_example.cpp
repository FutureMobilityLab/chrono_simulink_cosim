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

#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

// #include "chrono/utils/ChSocketCommunication.h"

#include "src/vehicle/WheeledVehicleForce.h"

#include <filesystem>
#include <iostream>

namespace chrono
{
  // =============================================================================
  class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON(unsigned int axle) const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual ChContactMethod ContactMethod() const = 0;
};

class Sedan_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Sedan"; }
    virtual std::string VehicleJSON() const override { return "sedan_force/vehicle/Sedan_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "sedan_force/tire/Sedan_RigidTire.json";
        return "sedan_force/tire/Sedan_TMeasyTire.json";
        // return "sedan_force/tire/Sedan_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override {
        ////return "sedan_force/powertrain/Sedan_EngineSimpleMap.json";
        return "sedan_force/powertrain/Sedan_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        ////return "sedan_force/powertrain/Sedan_AutomaticTransmissionSimpleMap.json";
        return "sedan_force/powertrain/Sedan_ManualTransmissionShafts.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};
} // namespace chrono

// =============================================================================

int main(int argc, char *argv[])
{
  // Create vehicle model.
  auto vehicle_model = chrono::Sedan_Model();

  // JSON files for terrain.v std::string
  const std::string rigidterrain_file("terrain/RigidPlane.json");

  // Initial vehicle position and orientation
  const chrono::ChVector3<> initLoc(0, 0, 0.5);
  const double initYaw = 20 * chrono::CH_DEG_TO_RAD;

  // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
  const auto chassis_vis_type = chrono::vehicle::VisualizationType::MESH;
  const auto suspension_vis_type = chrono::vehicle::VisualizationType::PRIMITIVES;
  const auto steering_vis_type = chrono::vehicle::VisualizationType::PRIMITIVES;
  const auto wheel_vis_type = chrono::vehicle::VisualizationType::MESH;
  const auto tire_vis_type = chrono::vehicle::VisualizationType::MESH;

  // Point on chassis tracked by the camera
  const chrono::ChVector3<> trackPoint(0.0, 0.0, 1.75);

  // Simulation step size.
  const double step_size = 2e-3;
  const double tire_step_size = 1e-3;

  // // The first item in argv is the path to current executable. Use this to set
  // // the Chrono Data Directory.
  // std::filesystem::path path(argv[0]);
  // std::filesystem::path grandparent_path = path.parent_path().parent_path();
  // std::filesystem::path data_dir_path(grandparent_path.string());
  // data_dir_path.append("data").append("");
  // std::filesystem::path veh_data_path(data_dir_path.string());
  // veh_data_path.append("vehicle").append("");
  // SetChronoDataPath(data_dir_path.string());
  // SetDataPath(veh_data_path.string());

  // --------------
  // Create systems
  // --------------

  // Create the vehicle system
  const std::string data_file = chrono::vehicle::GetDataFile(
      vehicle_model.VehicleJSON());
  chrono::vehicle::WheeledVehicleForce car(data_file,
                                           vehicle_model.ContactMethod());
  car.Initialize(chrono::ChCoordsys<>(initLoc, chrono::QuatFromAngleZ(initYaw)));
  car.GetChassis()->SetFixed(false);
  car.SetChassisVisualizationType(chassis_vis_type);
  car.SetChassisRearVisualizationType(chassis_vis_type);
  car.SetSubchassisVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car.SetSuspensionVisualizationType(suspension_vis_type);
  car.SetSteeringVisualizationType(steering_vis_type);
  car.SetWheelVisualizationType(wheel_vis_type);
  // car.LockAxleDifferential(0, false);

  // Create and initialize the powertrain system
  auto engine = chrono::vehicle::ReadEngineJSON(chrono::vehicle::GetDataFile(vehicle_model.EngineJSON()));
  auto transmission = chrono::vehicle::ReadTransmissionJSON(chrono::vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
  auto powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
  car.InitializePowertrain(powertrain);

  // Create and initialize the tires
  for (unsigned int i = 0; i < car.GetNumberAxles(); i++)
  {
    for (auto &wheel : car.GetAxle(i)->GetWheels())
    {
      auto tire = chrono::vehicle::ReadTireJSON(
          chrono::vehicle::GetDataFile(vehicle_model.TireJSON(i)));
      car.InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  // Containing system
  auto system = car.GetSystem();

    // Associate a collision system
  system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

  // Create the terrain (from JSON file)
  chrono::vehicle::RigidTerrain terrain(system, chrono::vehicle::GetDataFile(rigidterrain_file));
  terrain.Initialize();

  // Create the vehicle Irrlicht interface
  auto vis = chrono_types::make_shared<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("Rack and pinion demo");
  vis->SetChaseCamera(trackPoint, vehicle_model.CameraDistance(), 0.5);
  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(&car);

  // Create the interactive driver
  auto driver = chrono_types::make_shared<chrono::vehicle::ChInteractiveDriverIRR>(*vis);
  driver->SetSteeringDelta(0.02);
  // driver->SetGains(10.0);
  driver->SetThrottleDelta(0.02);
  driver->SetBrakingDelta(0.06);
  driver->Initialize();

  // ------------------------
  // Initialize values for simulation loop.
  // ------------------------

  car.LogSubsystemTypes();
  std::cout << "\nVehicle mass: " << car.GetMass() << std::endl;
  std::cout << "\nWheelbase: " << car.GetWheelbase() << std::endl;
  std::cout << "\nFront Track: " << car.GetWheeltrack(0) << std::endl;
  std::cout << "\nRear Track: " << car.GetWheeltrack(1) << std::endl;

  car.LogSubsystemTypes();

  // Enabling realtime attempts to make the simulation run at real time, even
  // if the simulation can run faster.
  car.EnableRealtime(true);
  while (vis->Run())
  {
    double time = car.GetSystem()->GetChTime();

    // Render scene
    vis->BeginScene();
    vis->Render();
    vis->EndScene();

    // Get driver inputs
    chrono::vehicle::DriverInputs driver_inputs = driver->GetInputs();

    // Update modules (process inputs from other modules)
    driver->Synchronize(time);
    driver_inputs.m_steering *= 2.0;
    car.Synchronize(time, driver_inputs, terrain);
    terrain.Synchronize(time);
    vis->Synchronize(time, driver_inputs);

    // Advance simulation for one timestep for all modules
    driver->Advance(step_size);
    car.Advance(step_size);
    terrain.Advance(step_size);
    vis->Advance(step_size);
  }
  return 0;
}
