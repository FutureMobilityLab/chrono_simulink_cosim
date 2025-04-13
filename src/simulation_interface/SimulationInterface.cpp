#include "src/simulation_interface/SimulationInterface.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include <filesystem>
#include <iostream>

namespace simulation_interface {

// SimulationInterface::SimulationInterface(const char* config_file) {
//   std::cout << "Calling simulation_interface: ";
//   std::cout << config_file << "\n";
//   this->config_file = config_file;
// }

// void SimulationInterface::step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
//   std::cout << "Calling SimulationInterface::step: ";
//   for (size_t i=0; i <= 2; i++) {
//     std::cout << "[" << i << "] = " << input[i] << "\n";
//   }
//   output[Output::SUM] = input[Input::THROTTLE] + input[Input::THROTTLE] + input[Input::BRAKE];
//   output[Output::DIFF] = input[Input::THROTTLE] + input[Input::THROTTLE] + input[Input::BRAKE];
// }

SimulationInterface::SimulationInterface(
  const char* vehicle_model_name
) {
  if (std::strcmp(vehicle_model_name, "sedan") == 0) {
    vehicle_model_ = new simulation_interface::Sedan_Model();
  } else {
    std::cerr << "Vehicle model name: " << vehicle_model_name 
              << " not supported. Check for typos.";
    throw(std::invalid_argument("Invalid vehicle model name."));
  }

  chrono::vehicle::SetDataPath("C:\\Users\\15309\\Project_Chrono\\chrono_simulink_cosim\\data\\vehicle\\");
  const std::string data_file = chrono::vehicle::GetDataFile(
      vehicle_model_->VehicleJSON());
  // const std::string data_file = "C:\\Users\\15309\\Project_Chrono\\chrono_simulink_cosim\\data\\vehicle/" + 
  //   vehicle_model_->VehicleJSON();
  std::cout << "data_file: " << data_file << "\n";
  car_ = new chrono::vehicle::WheeledVehicleForce(
    data_file,
    vehicle_model_->ContactMethod());

  const chrono::ChVector3<> initLoc(0, 0, 0.5);
  const double initYaw = 0.0 * chrono::CH_DEG_TO_RAD;
  car_->Initialize(chrono::ChCoordsys<>(initLoc, chrono::QuatFromAngleZ(initYaw)));
  car_->GetChassis()->SetFixed(false);

  const auto chassis_vis_type = chrono::vehicle::VisualizationType::MESH;
  car_->SetChassisVisualizationType(chassis_vis_type);
  car_->SetChassisRearVisualizationType(chassis_vis_type);
  car_->SetSubchassisVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car_->SetSuspensionVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car_->SetSteeringVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  car_->SetWheelVisualizationType(chrono::vehicle::VisualizationType::MESH);
  
  const auto engine = chrono::vehicle::ReadEngineJSON(
    chrono::vehicle::GetDataFile(vehicle_model_->EngineJSON()));
  const auto transmission = chrono::vehicle::ReadTransmissionJSON(
    chrono::vehicle::GetDataFile(vehicle_model_->TransmissionJSON()));
  const auto powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
  car_->InitializePowertrain(powertrain);
  // car_.LockAxleDifferential(0, false);

  const auto tire_vis_type = chrono::vehicle::VisualizationType::MESH;
  for (unsigned int i = 0; i < car_->GetNumberAxles(); i++)
  {
    for (auto &wheel : car_->GetAxle(i)->GetWheels())
    {
      auto tire = chrono::vehicle::ReadTireJSON(
          chrono::vehicle::GetDataFile(vehicle_model_->TireJSON(i)));
      car_->InitializeTire(tire, wheel, tire_vis_type);
    }
  }

  auto system = car_->GetSystem();
  system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

  const std::string rigidterrain_file("terrain/RigidPlane.json");
  terrain_ = new chrono::vehicle::RigidTerrain(system, chrono::vehicle::GetDataFile(rigidterrain_file));
  terrain_->Initialize();

  car_->LogSubsystemTypes();
  std::cout << "\nVehicle mass: " << car_->GetMass() << std::endl;
  std::cout << "\nWheelbase: " << car_->GetWheelbase() << std::endl;
  std::cout << "\nFront Track: " << car_->GetWheeltrack(0) << std::endl;
  std::cout << "\nRear Track: " << car_->GetWheeltrack(1) << std::endl;
  car_->EnableRealtime(false);
}

SimulationInterface::~SimulationInterface() {
  // TODO(tvidano): This causes EXCEPTION_ACCESS_VIOLATION in Julia when trying
  // to destroy ChRackPinionForce.

  // if (car_) {
  //   delete car_;
  //   car_ = nullptr;
  // }

  if (terrain_) {
    delete terrain_;
    terrain_ = nullptr;
  }

  if (vehicle_model_) {
    delete vehicle_model_;
    vehicle_model_ = nullptr;
  }
}

void SimulationInterface::Step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
    double time = car_->GetSystem()->GetChTime();

  if (vis_) {
    vis_->BeginScene();

    // TODO(tvidano): This is the line that causes problems in Julia. It appears
    // that it otherwise works. I have traced the issue to
    // ChVehicleVisualSystemIrrlicht::renderTextBox. However, I cannot tell
    // which line causes the problem. I can try the VSG renderer but that
    // requires adding the VSG dependency.
    vis_->Render();
    vis_->EndScene();

  }

  chrono::vehicle::DriverInputs driver_inputs;
  if (driver_) {
      driver_inputs = driver_->GetInputs();
      driver_->Synchronize(time);
      driver_->Advance(step_size_);
      driver_inputs.m_steering *= 2.0;
    } else {
      driver_inputs.m_steering = input[Input::STEERING];
      driver_inputs.m_throttle = 1.0; //input[Input::THROTTLE];
      driver_inputs.m_braking = input[Input::BRAKE];
  }
  std::cout << "Throttle: " << driver_inputs.m_throttle << "\t";

  car_->Synchronize(time, driver_inputs, *terrain_);
  terrain_->Synchronize(time);
  if (vis_) {
    vis_->Synchronize(time, driver_inputs);
    vis_->Advance(step_size_);
  }

  car_->Advance(step_size_);
  terrain_->Advance(step_size_);

  // Populate output signals
  output[Output::CHASSIS_POS_X] = car_->GetChassis()->GetPos().x();
  output[Output::CHASSIS_POS_Y] = car_->GetChassis()->GetPos().y();
  output[Output::CHASSIS_POS_Z] = car_->GetChassis()->GetPos().z();

  output[Output::CHASSIS_ORIENT_X] = car_->GetChassisBody()->GetRot().GetCardanAnglesXYZ().x();
  output[Output::CHASSIS_ORIENT_Y] = car_->GetChassisBody()->GetRot().GetCardanAnglesXYZ().y();
  output[Output::CHASSIS_ORIENT_Z] = car_->GetChassisBody()->GetRot().GetCardanAnglesXYZ().z();

  const auto chassis_frame = car_->GetChassisBody()->GetCoordsys();
  const auto pos_dt = car_->GetChassisBody()->GetPosDt();
  output[Output::CHASSIS_VEL_X] = chassis_frame.TransformDirectionParentToLocal(pos_dt).x();
  std::cout << "velocity x: " << chassis_frame.TransformDirectionParentToLocal(pos_dt).x() << "\n";
  output[Output::CHASSIS_VEL_Y] = chassis_frame.TransformDirectionParentToLocal(pos_dt).y();
  output[Output::CHASSIS_VEL_Z] = chassis_frame.TransformDirectionParentToLocal(pos_dt).z();

  output[Output::CHASSIS_ANG_VEL_X] = car_->GetChassisBody()->GetAngVelLocal().x();
  output[Output::CHASSIS_ANG_VEL_Y] = car_->GetChassisBody()->GetAngVelLocal().y();
  output[Output::CHASSIS_ANG_VEL_Z] = car_->GetChassisBody()->GetAngVelLocal().z();

  const auto pos_dtdt = car_->GetChassisBody()->GetLinAcc();
  output[Output::CHASSIS_ACC_X] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).x();
  output[Output::CHASSIS_ACC_Y] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).y();
  output[Output::CHASSIS_ACC_Z] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).z();

  output[Output::CHASSIS_ANG_ACC_X] = car_->GetChassisBody()->GetAngAccLocal().x();
  output[Output::CHASSIS_ANG_ACC_Y] = car_->GetChassisBody()->GetAngAccLocal().y();
  output[Output::CHASSIS_ANG_ACC_Z] = car_->GetChassisBody()->GetAngAccLocal().z();

  output[Output::WHEEL_ANG_VEL_FL] = car_->GetSpindleOmega(0, chrono::vehicle::VehicleSide::LEFT);
  output[Output::WHEEL_ANG_VEL_FR] = car_->GetSpindleOmega(0, chrono::vehicle::VehicleSide::RIGHT);
  output[Output::WHEEL_ANG_VEL_RL] = car_->GetSpindleOmega(1, chrono::vehicle::VehicleSide::LEFT);
  output[Output::WHEEL_ANG_VEL_RR] = car_->GetSpindleOmega(1, chrono::vehicle::VehicleSide::RIGHT);

  output[Output::TIRE_LONG_SLIP_FL] = car_->GetTire(0, chrono::vehicle::VehicleSide::LEFT)->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_FR] = car_->GetTire(0, chrono::vehicle::VehicleSide::RIGHT)->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_RL] = car_->GetTire(1, chrono::vehicle::VehicleSide::LEFT)->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_RR] = car_->GetTire(1, chrono::vehicle::VehicleSide::RIGHT)->GetLongitudinalSlip();

  // Add lateral slip angles
  output[Output::TIRE_LAT_SLIP_FL] = car_->GetTire(0, chrono::vehicle::VehicleSide::LEFT)->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_FR] = car_->GetTire(0, chrono::vehicle::VehicleSide::RIGHT)->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_RL] = car_->GetTire(1, chrono::vehicle::VehicleSide::LEFT)->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_RR] = car_->GetTire(1, chrono::vehicle::VehicleSide::RIGHT)->GetSlipAngle();

  // Add tire forces in tire frame
  auto tire_frame = chrono::ChCoordsys<>();
  const auto force_fl = car_->GetTire(0, chrono::vehicle::VehicleSide::LEFT)->ReportTireForceLocal(terrain_, tire_frame);
  const auto force_fr = car_->GetTire(0, chrono::vehicle::VehicleSide::RIGHT)->ReportTireForceLocal(terrain_, tire_frame);
  const auto force_rl = car_->GetTire(1, chrono::vehicle::VehicleSide::LEFT)->ReportTireForceLocal(terrain_, tire_frame);
  const auto force_rr = car_->GetTire(1, chrono::vehicle::VehicleSide::RIGHT)->ReportTireForceLocal(terrain_, tire_frame);

  output[Output::TIRE_FORCE_LONG_FL] = force_fl.force.x();
  output[Output::TIRE_FORCE_LONG_FR] = force_fr.force.x();
  output[Output::TIRE_FORCE_LONG_RL] = force_rl.force.x();
  output[Output::TIRE_FORCE_LONG_RR] = force_rr.force.x();

  output[Output::TIRE_FORCE_LAT_FL] = force_fl.force.y();
  output[Output::TIRE_FORCE_LAT_FR] = force_fr.force.y();
  output[Output::TIRE_FORCE_LAT_RL] = force_rl.force.y();
  output[Output::TIRE_FORCE_LAT_RR] = force_rr.force.y();

  output[Output::TIRE_FORCE_VERT_FL] = force_fl.force.z();
  output[Output::TIRE_FORCE_VERT_FR] = force_fr.force.z();
  output[Output::TIRE_FORCE_VERT_RL] = force_rl.force.z();
  output[Output::TIRE_FORCE_VERT_RR] = force_rr.force.z();

  // Add wheel torques
  output[Output::WHEEL_TORQUE_DRIVE_FL] = car_->GetDriveline()->GetSpindleTorque(0, chrono::vehicle::VehicleSide::LEFT);
  output[Output::WHEEL_TORQUE_DRIVE_FR] = car_->GetDriveline()->GetSpindleTorque(0, chrono::vehicle::VehicleSide::RIGHT);
  output[Output::WHEEL_TORQUE_DRIVE_RL] = car_->GetDriveline()->GetSpindleTorque(1, chrono::vehicle::VehicleSide::LEFT);
  output[Output::WHEEL_TORQUE_DRIVE_RR] = car_->GetDriveline()->GetSpindleTorque(1, chrono::vehicle::VehicleSide::RIGHT);

  output[Output::WHEEL_TORQUE_BRAKE_FL] = car_->GetBrake(0, chrono::vehicle::VehicleSide::LEFT)->GetBrakeTorque();
  output[Output::WHEEL_TORQUE_BRAKE_FR] = car_->GetBrake(0, chrono::vehicle::VehicleSide::RIGHT)->GetBrakeTorque();
  output[Output::WHEEL_TORQUE_BRAKE_RL] = car_->GetBrake(1, chrono::vehicle::VehicleSide::LEFT)->GetBrakeTorque();
  output[Output::WHEEL_TORQUE_BRAKE_RR] = car_->GetBrake(1, chrono::vehicle::VehicleSide::RIGHT)->GetBrakeTorque();

  // Add steering pinion angle
  output[Output::STEERING_PINION_ANGLE] = car_->GetPinionAngle();

  // Road wheels steer angle (angle made between wheel normal axis and chassis y plane).
  const auto wheel_normal_fl = car_->GetWheel(0,chrono::vehicle::VehicleSide::LEFT)->GetState().rot.GetAxisY();
  const auto normal_fl = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_fl);
  output[Output::WHEEL_STEER_ANG_FL] = std::atan2(normal_fl.x(), normal_fl.y());
  const auto wheel_normal_fr = car_->GetWheel(0,chrono::vehicle::VehicleSide::RIGHT)->GetState().rot.GetAxisY();
  const auto normal_fr = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_fr);
  output[Output::WHEEL_STEER_ANG_FR] = std::atan2(normal_fr.x(), normal_fr.y());
  const auto wheel_normal_rl = car_->GetWheel(1,chrono::vehicle::VehicleSide::LEFT)->GetState().rot.GetAxisY();
  const auto normal_rl = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_rl);
  output[Output::WHEEL_STEER_ANG_RL] = std::atan2(normal_rl.x(),normal_rl.y());
  const auto wheel_normal_rr = car_->GetWheel(1,chrono::vehicle::VehicleSide::RIGHT)->GetState().rot.GetAxisY();
  const auto normal_rr = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_rr);
  output[Output::WHEEL_STEER_ANG_RR] = std::atan2(normal_rr.x(),normal_rr.y());
}

}
