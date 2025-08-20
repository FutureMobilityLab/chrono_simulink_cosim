#include "src/simulation_interface/SimulationInterface.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include <filesystem>
#include <iostream>
#include <cmath>
#include <limits>

namespace simulation_interface {

// Implementation of TerrainInterface
TerrainInterface::TerrainInterface(chrono::vehicle::WheeledVehicleForce* vehicle) : m_vehicle(vehicle) {
  // Initialize with default values
  for (int i = 0; i < 4; i++) {
    m_height[i] = 0.0;
    m_normal[i] = chrono::ChVector3d(0, 0, 1); // Default normal points up
    m_friction[i] = 0.8; // Default friction coefficient
  }
}

TerrainInterface::~TerrainInterface() {}

double TerrainInterface::GetHeight(const chrono::ChVector3d& loc) const {
  int idx = FindClosestWheel(loc);
  return m_height[idx];
}

chrono::ChVector3d TerrainInterface::GetNormal(const chrono::ChVector3d& loc) const {
  int idx = FindClosestWheel(loc);
  return m_normal[idx];
}

float TerrainInterface::GetCoefficientFriction(const chrono::ChVector3d& loc) const {
  int idx = FindClosestWheel(loc);
  return static_cast<float>(m_friction[idx]);
}

void TerrainInterface::SetTerrainHeight(int wheel_idx, double height) {
  if (wheel_idx >= 0 && wheel_idx < 4) {
    m_height[wheel_idx] = height;
  }
}

void TerrainInterface::SetTerrainNormal(int wheel_idx, double x, double y, double z) {
  if (wheel_idx >= 0 && wheel_idx < 4) {
    m_normal[wheel_idx] = chrono::ChVector3d(x, y, z);
    m_normal[wheel_idx].Normalize();
  }
}

void TerrainInterface::SetTerrainFriction(int wheel_idx, double mu) {
  if (wheel_idx >= 0 && wheel_idx < 4) {
    m_friction[wheel_idx] = mu;
  }
}

int TerrainInterface::FindClosestWheel(const chrono::ChVector3d& loc) const {
  std::vector<chrono::ChVector3d> wheel_positions;
  wheel_positions.push_back(m_vehicle->GetWheel(0, chrono::vehicle::VehicleSide::LEFT)->GetState().pos);
  wheel_positions.push_back(m_vehicle->GetWheel(0, chrono::vehicle::VehicleSide::RIGHT)->GetState().pos);
  wheel_positions.push_back(m_vehicle->GetWheel(1, chrono::vehicle::VehicleSide::LEFT)->GetState().pos);
  wheel_positions.push_back(m_vehicle->GetWheel(1, chrono::vehicle::VehicleSide::RIGHT)->GetState().pos);

  double min_distance = std::numeric_limits<double>::max();
  int closest_wheel = -1;
  for (int i = 0; i < 4; i++) {
    double distance = (loc - wheel_positions[i]).Length();
    if (distance < min_distance) {
      min_distance = distance;
      closest_wheel = i;
    }
  }
  return closest_wheel;
}

SimulationInterface::SimulationInterface(
  const char* vehicle_model_name
) {
  std::cout << "Creating SimulationInterface instance.\n";
  if (std::strcmp(vehicle_model_name, "sedan") == 0) {
    vehicle_model_ = new simulation_interface::Sedan_Model();
  } else if (std::strcmp(vehicle_model_name, "hmmwv") == 0) {
    vehicle_model_ = new simulation_interface::HMMWV_Model();
  } else {
    std::cerr << "Vehicle model name: " << vehicle_model_name 
              << " not supported. Check for typos.";
    throw(std::invalid_argument("Invalid vehicle model name."));
  }

  chrono::vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
  const std::string data_file = chrono::vehicle::GetDataFile(
      vehicle_model_->VehicleJSON());
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

  // Create our custom terrain instead of RigidTerrain
  terrain_ = std::make_shared<TerrainInterface>(car_);

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

  // if (terrain_) {
  //   delete terrain_;
  //   terrain_ = nullptr;
  // }

  if (vehicle_model_) {
    delete vehicle_model_;
    vehicle_model_ = nullptr;
  }
  std::cout << "Destroying SimulationInterface instance.";
}

void SimulationInterface::Step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
  double time = car_->GetSystem()->GetChTime();
  std::cout << "Step() with time: " << time << "\n";

  if (vis_) {
    vis_->BeginScene();

    vis_->EnableStats(false);
    
    // TODO(tvidano): This is the line that causes problems in Julia. It appears
    // that it otherwise works. I have traced the issue to
    // ChVehicleVisualSystemIrrlicht::renderTextBox. However, I cannot tell
    // which line causes the problem. I can try the VSG renderer but that
    // requires adding the VSG dependency.
    vis_->Render();
    
    vis_->EndScene();
  }

  // Process terrain input parameters
  // Process heights
  terrain_->SetTerrainHeight(FL, input[Input::TERRAIN_HEIGHT_FL]);
  terrain_->SetTerrainHeight(FR, input[Input::TERRAIN_HEIGHT_FR]);
  terrain_->SetTerrainHeight(RL, input[Input::TERRAIN_HEIGHT_RL]);
  terrain_->SetTerrainHeight(RR, input[Input::TERRAIN_HEIGHT_RR]);
  
  // Process normals
  terrain_->SetTerrainNormal(FL, 
    input[Input::TERRAIN_NORMAL_X_FL], 
    input[Input::TERRAIN_NORMAL_Y_FL], 
    input[Input::TERRAIN_NORMAL_Z_FL]);
  terrain_->SetTerrainNormal(FR, 
    input[Input::TERRAIN_NORMAL_X_FR], 
    input[Input::TERRAIN_NORMAL_Y_FR], 
    input[Input::TERRAIN_NORMAL_Z_FR]);
  terrain_->SetTerrainNormal(RL, 
    input[Input::TERRAIN_NORMAL_X_RL], 
    input[Input::TERRAIN_NORMAL_Y_RL], 
    input[Input::TERRAIN_NORMAL_Z_RL]);
  terrain_->SetTerrainNormal(RR, 
    input[Input::TERRAIN_NORMAL_X_RR], 
    input[Input::TERRAIN_NORMAL_Y_RR], 
    input[Input::TERRAIN_NORMAL_Z_RR]);
  
  // Process friction coefficients
  terrain_->SetTerrainFriction(FL, input[Input::TERRAIN_MU_FL]);
  terrain_->SetTerrainFriction(FR, input[Input::TERRAIN_MU_FR]);
  terrain_->SetTerrainFriction(RL, input[Input::TERRAIN_MU_RL]);
  terrain_->SetTerrainFriction(RR, input[Input::TERRAIN_MU_RR]);

  chrono::vehicle::DriverInputs driver_inputs;
  if (driver_) {
      driver_inputs = driver_->GetInputs();
      driver_->Synchronize(time);
      driver_->Advance(step_size_);
      driver_inputs.m_steering *= 4.0;
    } else {
      driver_inputs.m_steering = input[Input::STEERING];
      driver_inputs.m_throttle = input[Input::THROTTLE];
      driver_inputs.m_braking = input[Input::BRAKE];
  }

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
  const auto force_fl = car_->GetTire(0, chrono::vehicle::VehicleSide::LEFT)->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_fr = car_->GetTire(0, chrono::vehicle::VehicleSide::RIGHT)->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_rl = car_->GetTire(1, chrono::vehicle::VehicleSide::LEFT)->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_rr = car_->GetTire(1, chrono::vehicle::VehicleSide::RIGHT)->ReportTireForceLocal(terrain_.get(), tire_frame);

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

  // Add query points to output
  auto query_point = car_->GetWheel(0, chrono::vehicle::VehicleSide::LEFT)->GetState().pos;
  output[Output::QUERY_POINT_X_FL] = query_point.x();
  output[Output::QUERY_POINT_Y_FL] = query_point.y();
  output[Output::QUERY_POINT_Z_FL] = query_point.z();
  
  query_point = car_->GetWheel(0, chrono::vehicle::VehicleSide::RIGHT)->GetState().pos;
  output[Output::QUERY_POINT_X_FR] = query_point.x();
  output[Output::QUERY_POINT_Y_FR] = query_point.y();
  output[Output::QUERY_POINT_Z_FR] = query_point.z();
  
  query_point = car_->GetWheel(1, chrono::vehicle::VehicleSide::LEFT)->GetState().pos;
  output[Output::QUERY_POINT_X_RL] = query_point.x();
  output[Output::QUERY_POINT_Y_RL] = query_point.y();
  output[Output::QUERY_POINT_Z_RL] = query_point.z();
  
  query_point = car_->GetWheel(1, chrono::vehicle::VehicleSide::RIGHT)->GetState().pos;
  output[Output::QUERY_POINT_X_RR] = query_point.x();
  output[Output::QUERY_POINT_Y_RR] = query_point.y();
  output[Output::QUERY_POINT_Z_RR] = query_point.z();

  output[Output::SIM_TIME] = time;
}

double SimulationInterface::GetSimTime() {
  return car_->GetSystem()->GetChTime();
}

}
