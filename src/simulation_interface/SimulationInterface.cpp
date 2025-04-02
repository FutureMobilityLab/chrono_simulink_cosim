#include "src/simulation_interface/SimulationInterface.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include <filesystem>
#include <iostream>

namespace simulation_interface {

// Simulation_Interface::Simulation_Interface(const char* config_file) {
//   std::cout << "Calling simulation_interface: ";
//   std::cout << config_file << "\n";
//   this->config_file = config_file;
// }

// void Simulation_Interface::step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
//   std::cout << "Calling Simulation_Interface::step: ";
//   for (size_t i=0; i <= 2; i++) {
//     std::cout << "[" << i << "] = " << input[i] << "\n";
//   }
//   output[Output::SUM] = input[Input::THROTTLE] + input[Input::THROTTLE] + input[Input::BRAKE];
//   output[Output::DIFF] = input[Input::THROTTLE] + input[Input::THROTTLE] + input[Input::BRAKE];
// }

Simulation_Interface::Simulation_Interface(
  const char* vehicle_model_name
) {
  Vehicle_Model* vehicle_model = nullptr;
  if (std::strcmp(vehicle_model_name, "sedan") == 0) {
    vehicle_model = new simulation_interface::Sedan_Model();
  } else {
    std::cerr << "Vehicle model name: " << vehicle_model_name 
              << " not supported. Check for typos.";
    throw(std::invalid_argument("Invalid vehicle model name."));
  }

  chrono::vehicle::SetDataPath("C:\\Users\\15309\\Project_Chrono\\chrono_simulink_cosim\\data\\vehicle\\");
  const std::string data_file = chrono::vehicle::GetDataFile(
      vehicle_model->VehicleJSON());
  // const std::string data_file = "C:\\Users\\15309\\Project_Chrono\\chrono_simulink_cosim\\data\\vehicle/" + 
  //   vehicle_model->VehicleJSON();
  std::cout << "data_file: " << data_file << "\n";
  this->car = new chrono::vehicle::WheeledVehicleForce(
    data_file,
    vehicle_model->ContactMethod());

  // Store the vehicle model for cleanup in destructor
  this->vehicle_model = vehicle_model;

  const chrono::ChVector3<> initLoc(0, 0, 0.5);
  const double initYaw = 0.0 * chrono::CH_DEG_TO_RAD;
  car->Initialize(chrono::ChCoordsys<>(initLoc, chrono::QuatFromAngleZ(initYaw)));
  car->GetChassis()->SetFixed(false);

  const auto chassis_vis_type = chrono::vehicle::VisualizationType::MESH;
  car->SetChassisVisualizationType(chassis_vis_type);
  car->SetChassisRearVisualizationType(chassis_vis_type);
  car->SetSubchassisVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car->SetSuspensionVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car->SetSteeringVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car->SetWheelVisualizationType(chrono::vehicle::VisualizationType::MESH);
  
  const auto engine = chrono::vehicle::ReadEngineJSON(
    chrono::vehicle::GetDataFile(vehicle_model->EngineJSON()));
  const auto transmission = chrono::vehicle::ReadTransmissionJSON(
    chrono::vehicle::GetDataFile(vehicle_model->TransmissionJSON()));
  const auto powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
  car->InitializePowertrain(powertrain);
  // car.LockAxleDifferential(0, false);

  const auto tire_vis_type = chrono::vehicle::VisualizationType::MESH;
  for (unsigned int i = 0; i < car->GetNumberAxles(); i++)
  {
    for (auto &wheel : car->GetAxle(i)->GetWheels())
    {
      auto tire = chrono::vehicle::ReadTireJSON(
          chrono::vehicle::GetDataFile(vehicle_model->TireJSON(i)));
      car->InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  auto system = car->GetSystem();
  system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

  const std::string rigidterrain_file("terrain/RigidPlane.json");
  terrain = new chrono::vehicle::RigidTerrain(system, chrono::vehicle::GetDataFile(rigidterrain_file));
  terrain->Initialize();

  vis = chrono_types::make_shared<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("Rack and pinion demo");
  const chrono::ChVector3<> trackPoint(0.0, 0.0, 1.75);
  vis->SetChaseCamera(trackPoint, vehicle_model->CameraDistance(), 0.5);
  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(car);
  
  driver = chrono_types::make_shared<chrono::vehicle::ChInteractiveDriverIRR>(*vis);
  driver->SetSteeringDelta(0.02);
  // driver->SetGains(10.0);
  driver->SetThrottleDelta(0.02);
  driver->SetBrakingDelta(0.06);
  driver->Initialize();

  car->LogSubsystemTypes();
  std::cout << "\nVehicle mass: " << car->GetMass() << std::endl;
  std::cout << "\nWheelbase: " << car->GetWheelbase() << std::endl;
  std::cout << "\nFront Track: " << car->GetWheeltrack(0) << std::endl;
  std::cout << "\nRear Track: " << car->GetWheeltrack(1) << std::endl;
  car->EnableRealtime(true);
}

Simulation_Interface::~Simulation_Interface() {
  if (car) {
    delete car;
    car = nullptr;
  }

  if (terrain) {
    delete terrain;
    terrain = nullptr;
  }

  if (vehicle_model) {
    delete vehicle_model;
    vehicle_model = nullptr;
  }
}

void Simulation_Interface::Step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
  std::cout << "Simulation_Interface::Step()\n";
  double time = car->GetSystem()->GetChTime();

  std::cout << "BeginScene\n";
  vis->BeginScene();
  std::cout << "Render\n";
  // TODO(tvidano): This is the line that causes problems in Julia. It appears that it otherwise works.
  vis->Render();
  std::cout << "EndScene\n";
  vis->EndScene();

  // Get driver inputs
  std::cout << "Getting inputs\n";
  chrono::vehicle::DriverInputs driver_inputs = driver->GetInputs();
  
  // // Override with provided inputs from array
  // driver_inputs.m_steering = input[Input::STEERING];
  // driver_inputs.m_throttle = input[Input::THROTTLE];
  // driver_inputs.m_braking = input[Input::BRAKE];

  // Update modules (process inputs from other modules)
  std::cout << "Synchronizing driver\n";
  driver->Synchronize(time);
  driver_inputs.m_steering *= 2.0;
  std::cout << "Synchronizing car\n";
  car->Synchronize(time, driver_inputs, *terrain);
  std::cout << "Synchronizing terrain\n";
  terrain->Synchronize(time);
  vis->Synchronize(time, driver_inputs);

  // Advance simulation for one timestep for all modules
  std::cout << "Advancing\n";
  driver->Advance(step_size);
  car->Advance(step_size);
  terrain->Advance(step_size);
  vis->Advance(step_size);
}

std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> Simulation_Interface::get_vis() {
  return vis;
}

}
