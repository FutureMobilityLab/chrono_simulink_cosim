#include "src/simulation_interface/SimulationInterface.h"
#include "src/utils/utils.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono/utils/ChConstants.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"

#include <filesystem>
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>

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

namespace {
  void PrintSolverType(chrono::ChSystem* system) {
    // For solver
    auto solver = system->GetSolver();
    auto solver_type = solver->GetType();
    std::cout << "Solver: ";
    switch(solver_type) {
      case chrono::ChSolver::Type::PSOR:
          std::cout << "PSOR" << std::endl; break;
      case chrono::ChSolver::Type::PSSOR:
        std::cout << "PSSOR" << std::endl; break;
      case chrono::ChSolver::Type::PJACOBI:
        std::cout << "PJACOBI" << std::endl; break;
      case chrono::ChSolver::Type::PMINRES:
          std::cout << "PMINRES" << std::endl; break;
      case chrono::ChSolver::Type::BARZILAIBORWEIN:
        std::cout << "BARZILAIBORWEIN\n"; break;
      case chrono::ChSolver::Type::APGD:
        std::cout << "APGD\n"; break;
      case chrono::ChSolver::Type::BICGSTAB:
            std::cout << "BiCGSTAB" << std::endl; break;
      case chrono::ChSolver::Type::GMRES:
          std::cout << "GMRES" << std::endl; break;
      case chrono::ChSolver::Type::SPARSE_LU:
          std::cout << "Sparse LU" << std::endl; break;
      case chrono::ChSolver::Type::SPARSE_QR:
          std::cout << "Sparse QR" << std::endl; break;
      default:
          std::cout << "Unknown/Other (enum: " << static_cast<int>(solver_type) << ")" << std::endl;
    }
  }

  void PrintTimeStepperType(chrono::ChSystem* system) {
    auto integrator = system->GetTimestepper();
    auto integrator_type = integrator->GetType();
    std::cout << "Integrator: ";
    switch(integrator_type) {
      case chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
          std::cout << "Euler Implicit Linearized" << std::endl; break;
      case chrono::ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
        std::cout << "Euler Implicit Projected" << std::endl; break;
      case chrono::ChTimestepper::Type::EULER_IMPLICIT:
        std::cout << "Euler Implicit" << std::endl; break;
      case chrono::ChTimestepper::Type::TRAPEZOIDAL:
        std::cout << "Trapezoidal" << std::endl; break;
      case chrono::ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
        std::cout << "Trapezoidal Linearized" << std::endl; break;
      case chrono::ChTimestepper::Type::HHT:
        std::cout << "HHT" << std::endl; break;
      case chrono::ChTimestepper::Type::HEUN:
        std::cout << "Heun" << std::endl; break;
      case chrono::ChTimestepper::Type::RUNGEKUTTA45:
        std::cout << "Runge Kutta 45" << std::endl; break;
      case chrono::ChTimestepper::Type::EULER_EXPLICIT:
        std::cout << "Euler Explicit" << std::endl; break;
      case chrono::ChTimestepper::Type::LEAPFROG:
        std::cout << "Leapfrog" << std::endl; break;
      case chrono::ChTimestepper::Type::NEWMARK:
        std::cout << "Newmark" << std::endl; break;
      default:
        std::cout << "Unknown/Other (enum: " << static_cast<int>(integrator_type) << ")" << std::endl;
    }
  }
}

SimulationInterface::SimulationInterface(
  const char* vehicle_model_name
) {
  if (std::strcmp(vehicle_model_name, "sedan") == 0) {
    // vehicle_model_ = new simulation_interface::Sedan_Model();
    vehicle_json_ = "sedan_force/vehicle/Sedan_Vehicle.json";
  } else if (std::strcmp(vehicle_model_name, "hmmwv") == 0) {
    // vehicle_model_ = new simulation_interface::HMMWV_Model();
    vehicle_json_ = "hmmwv/vehicle/HMMWV_Vehicle_Force.json";
  } else if (std::strcmp(vehicle_model_name, "ford_expedition_2003") == 0) {
    vehicle_json_ = "ford_expedition_2003/Vehicle_ford_expedition_2003.json";
  } else {
    std::cerr << "Vehicle model name: " << vehicle_model_name 
              << " not supported. Check for typos.";
    throw(std::invalid_argument("Invalid vehicle model name."));
  }

  chrono::vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
  // const std::string data_file = chrono::vehicle::GetDataFile(
  //     vehicle_model_->VehicleJSON());
  const std::string data_file = chrono::vehicle::GetDataFile(
      vehicle_json_);
  car_ = new chrono::vehicle::WheeledVehicleForce(
    data_file,
    chrono::ChContactMethod::SMC,
    true,
    true);

  const auto system = car_->GetSystem();
  // system->SetSolverType(chrono::ChSolver::Type::PMINRES);
  // const auto solver = system->GetSolver();
  // if (auto bb_solver = std::dynamic_pointer_cast<chrono::ChSolverBB>(solver)) {
  //   std::cout << "MaxIterations: " << bb_solver->GetMaxIterations() << "\n";
  //   bb_solver->SetMaxIterations(800);  // Increase iterations
  //   std:: cout << "Tolerance: " << bb_solver->GetTolerance() << "\n";
  //   bb_solver->SetTolerance(1e-12);    // Tighter tolerance
  // }

  // All comments are made with TMeasy tire.
  // This eliminates the chattering (1.39 real/sim)
  // system->SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  // This has chattering (0.4 real/sim) (no chattering with Pac89 tire)
  // system->SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
  // This has chattering (0.7 real/sim)
  // system->SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
  // This works and has (1.5 real/sim)
  // system->SetTimestepperType(chrono::ChTimestepper::Type::TRAPEZOIDAL);
  // This works and is fast (<0.5 real/sim)
  system->SetTimestepperType(chrono::ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED);
  // This fails.
  // system->SetTimestepperType(chrono::ChTimestepper::Type::HHT);
  // This fails: HHT: Reached minimum allowable step size.
  // auto timestepper = chrono_types::make_shared<chrono::ChTimestepperHHT>(system);
  // timestepper->SetMinStepSize(1e-10);
  // timestepper->SetMaxIters(4);
  // timestepper->SetAlpha(-0.1);
  // system->SetTimestepper(timestepper);
  // This fails.
  // system->SetTimestepperType(chrono::ChTimestepper::Type::HEUN);
  // This fails.
  // system->SetTimestepperType(chrono::ChTimestepper::Type::RUNGEKUTTA45);
  // This fails.
  // system->SetTimestepperType(chrono::ChTimestepper::Type::EULER_EXPLICIT);
  // This fails.
  // system->SetTimestepperType(chrono::ChTimestepper::Type::LEAPFROG);
  // This works (1.9 real/sim)
  // system->SetTimestepperType(chrono::ChTimestepper::Type::NEWMARK);

  PrintSolverType(system);
  PrintTimeStepperType(system);

  // auto hht_integrator = std::make_shared<chrono::ChTimestepperHHT>();
  // // hht_integrator->SetAlpha(-0.1);
  // // hht_integrator->SetMaxIters(20);
  // // hht_integrator->SetAbsTolerances(1e-6);
  // system->SetTimestepper(hht_integrator);

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
  
  // const auto engine = chrono::vehicle::ReadEngineJSON(
  //   chrono::vehicle::GetDataFile(vehicle_model_->EngineJSON()));
  // const auto transmission = chrono::vehicle::ReadTransmissionJSON(
  //   chrono::vehicle::GetDataFile(vehicle_model_->TransmissionJSON()));
  // const auto powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
  // car_->InitializePowertrain(powertrain);
  // car_.LockAxleDifferential(0, false);

  // const auto tire_vis_type = chrono::vehicle::VisualizationType::MESH;
  // for (unsigned int i = 0; i < car_->GetNumberAxles(); i++)
  // {
  //   for (auto &wheel : car_->GetAxle(i)->GetWheels())
  //   {
  //     auto tire = chrono::vehicle::ReadCustomTireJSON(
  //         chrono::vehicle::GetDataFile(vehicle_model_->TireJSON(i)));
  //     car_->InitializeTire(tire, wheel, tire_vis_type);
  //     tire->SetStepsize(tire_step_size_);
  //   }
  // }

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

  // if (vehicle_model_) {
  //   delete vehicle_model_;
  //   vehicle_model_ = nullptr;
  // }
}

void SimulationInterface::Step(const double input[Input::LENGTH], double output[Output::LENGTH]) {
  double time = car_->GetSystem()->GetChTime();

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

  // // Make a copy of input using memcpy (memset is for zeroing memory, memcpy copies)
  // double input[Input::LENGTH];
  // std::memset(input, 0, sizeof(double) * Input::LENGTH);
  // std::memcpy(input, input_copy, sizeof(double) * Input::LENGTH);
  // for (size_t i = 0; i < Input::LENGTH; ++i) {
  //   std::cout << "[" << i << "]: " << input[i] << " ";
  // }
  // std::cout << "\n";

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
  } else {
    driver_inputs.m_steering = input[Input::STEERING];
    driver_inputs.m_throttle = input[Input::THROTTLE];
    driver_inputs.m_braking = input[Input::BRAKE];
  }

  auto steering = car_->GetSteering(0);
  if (auto steering_rp = std::dynamic_pointer_cast<chrono::vehicle::RackPinion>(steering)) {
    // If input looks like an absolute pinion angle (abs > 1), scale it to the
    // normalized [-1,1] steering command expected by ChRackPinion using GetMaxAngle().
    double max_ang = steering_rp->GetMaxAngle();
    driver_inputs.m_steering = driver_inputs.m_steering / max_ang;
    // Clamp to [-1,1].
    if (driver_inputs.m_steering > 1.0)
      driver_inputs.m_steering = 1.0;
    else if (driver_inputs.m_steering < -1.0)
      driver_inputs.m_steering = -1.0;
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

  // Get tires.
  const auto tire_fl = car_->GetTire(0, chrono::vehicle::VehicleSide::LEFT);
  const auto tire_fr = car_->GetTire(0, chrono::vehicle::VehicleSide::RIGHT);
  const auto tire_rl = car_->GetTire(1, chrono::vehicle::VehicleSide::LEFT);
  const auto tire_rr = car_->GetTire(1, chrono::vehicle::VehicleSide::RIGHT);

  output[Output::TIRE_LONG_SLIP_FL] = tire_fl->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_FR] = tire_fr->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_RL] = tire_rl->GetLongitudinalSlip();
  output[Output::TIRE_LONG_SLIP_RR] = tire_rr->GetLongitudinalSlip();

  // Add lateral slip angles
  output[Output::TIRE_LAT_SLIP_FL] = tire_fl->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_FR] = tire_fr->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_RL] = tire_rl->GetSlipAngle();
  output[Output::TIRE_LAT_SLIP_RR] = tire_rr->GetSlipAngle();

  // Add tire forces in tire frame.
  auto tire_frame = chrono::ChCoordsys<>();
  const auto force_fl = tire_fl->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_fr = tire_fr->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_rl = tire_rl->ReportTireForceLocal(terrain_.get(), tire_frame);
  const auto force_rr = tire_rr->ReportTireForceLocal(terrain_.get(), tire_frame);

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
  
  // Front Left tire moments
  output[Output::TIRE_MOMENT_X_FL] = force_fl.moment.x();
  output[Output::TIRE_MOMENT_Y_FL] = force_fl.moment.y();
  output[Output::TIRE_MOMENT_Z_FL] = force_fl.moment.z();

  // Front Right tire moments  
  output[Output::TIRE_MOMENT_X_FR] = force_fr.moment.x();
  output[Output::TIRE_MOMENT_Y_FR] = force_fr.moment.y();
  output[Output::TIRE_MOMENT_Z_FR] = force_fr.moment.z();

  // Rear Left tire moments
  output[Output::TIRE_MOMENT_X_RL] = force_rl.moment.x();
  output[Output::TIRE_MOMENT_Y_RL] = force_rl.moment.y();
  output[Output::TIRE_MOMENT_Z_RL] = force_rl.moment.z();

  // Rear Right tire moments
  output[Output::TIRE_MOMENT_X_RR] = force_rr.moment.x();
  output[Output::TIRE_MOMENT_Y_RR] = force_rr.moment.y();
  output[Output::TIRE_MOMENT_Z_RR] = force_rr.moment.z();

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
  const auto steering_link = car_->GetSteering(0)->GetSteeringLink();
  const auto steering_force_abs = steering_link->GetAppliedForce();
  const auto link_frame_abs = steering_link->GetFrameRefToAbs();
  const auto steering_force_local = link_frame_abs.TransformDirectionParentToLocal(steering_force_abs);
  const auto acceleration = steering_link->GetPosDt2();
  const auto acceleration_local = link_frame_abs.TransformDirectionParentToLocal(acceleration);
  // if (std::abs(acceleration.x()) > 2 || std::abs(acceleration.y()) > 2 ||
  //     std::abs(acceleration.z()) > 2) {
  //   std::cout << "Time: " << time << ", Force: " << steering_force_local.z() << " Acc: " << acceleration.x()
  //             << ", " << acceleration.y() << ", " << acceleration.z() << "\n";
  // }
  output[Output::STEERING_PINION_ANGLE] = car_->GetPinionAngle();
  // output[Output::STEERING_PINION_ANGLE] = steering_force_local.z();
  // output[Output::STEERING_PINION_ANGLE] = link_frame_abs.TransformDirectionParentToLocal(steering_link->GetPos()).x();

  // Road wheels steer angle (angle made between wheel normal axis and chassis y plane).
  const auto wheel_normal_fl = car_->GetWheel(0,chrono::vehicle::VehicleSide::LEFT)->GetState().rot.GetAxisY();
  const auto normal_fl = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_fl);
  // output[Output::WHEEL_STEER_ANG_FL] = std::atan2(normal_fl.x(), normal_fl.y());
  const auto wheel_normal_fr = car_->GetWheel(0,chrono::vehicle::VehicleSide::RIGHT)->GetState().rot.GetAxisY();
  const auto normal_fr = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_fr);
  // output[Output::WHEEL_STEER_ANG_FR] = std::atan2(normal_fr.x(), normal_fr.y());
  const auto wheel_normal_rl = car_->GetWheel(1,chrono::vehicle::VehicleSide::LEFT)->GetState().rot.GetAxisY();
  const auto normal_rl = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_rl);
  // output[Output::WHEEL_STEER_ANG_RL] = std::atan2(normal_rl.x(),normal_rl.y());
  const auto wheel_normal_rr = car_->GetWheel(1,chrono::vehicle::VehicleSide::RIGHT)->GetState().rot.GetAxisY();
  const auto normal_rr = car_->GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal_rr);
  // output[Output::WHEEL_STEER_ANG_RR] = std::atan2(normal_rr.x(),normal_rr.y());

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

  // Validate output.
  for (size_t i = 0; i < Output::LENGTH; i++) {
    if (std::isnan(output[i])) {
      std::cerr << "SimulationInterface::Step NaN detected at output[" << i
                << "] at time " << time << std::endl;
      throw std::runtime_error("Got nan for output[" + std::to_string(i) + "].");
    }
  }

  // Verify vehicle has not rolled over.
  if (std::abs(output[Output::CHASSIS_ORIENT_X]) > chrono::CH_PI_2) {
    std::cerr << "Vehicle roll limit exceeded at time " << time
              << " roll=" << output[Output::CHASSIS_ORIENT_X] << std::endl;
    throw std::runtime_error("Vehicle rolled over with roll = " + std::to_string(output[Output::CHASSIS_ORIENT_X]) + " (axis-x).");
  }
  if (std::abs(output[Output::CHASSIS_ORIENT_Y]) > chrono::CH_PI_2) {
    std::cerr << "Vehicle pitch limit exceeded at time " << time
              << " pitch=" << output[Output::CHASSIS_ORIENT_Y] << std::endl;
    throw std::runtime_error("Vehicle rolled over with pitch = " + std::to_string(output[Output::CHASSIS_ORIENT_Y]) + " (axis-y).");
  }

  // Verify vehicle did not fall through ground.
  const double min_height = std::min({input[Input::TERRAIN_HEIGHT_FL], input[Input::TERRAIN_HEIGHT_FR],
                                        input[Input::TERRAIN_HEIGHT_RL], input[Input::TERRAIN_HEIGHT_RR]});
  if (output[Output::CHASSIS_POS_Z] <  min_height - 0.5) {
    std::cerr << "Vehicle below terrain at time " << time
              << " chassis_z=" << output[Output::CHASSIS_POS_Z]
              << " min_height=" << min_height << std::endl;
    throw std::runtime_error("Vehicle fell through the ground.");
  }
}

double SimulationInterface::GetSimTime() {
  return car_->GetSystem()->GetChTime();
}

}
