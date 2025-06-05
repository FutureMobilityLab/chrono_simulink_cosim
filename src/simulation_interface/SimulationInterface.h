#ifndef SIMULATIONINTERFACE_H
#define SIMULATIONINTERFACE_H

// Remove custom export macros
// #if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
//     #if defined( SIM_INTERFACE_EXPORTS )
//         #define SIM_INTERFACE_API __declspec(dllexport)
//     #else
//         #define SIM_INTERFACE_API __declspec(dllimport)
//     #endif
// #else
//     #define SIM_INTERFACE_API
// #endif

#include "chrono/physics/ChContactMaterial.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "src/vehicle/WheeledVehicleForce.h"

#include <string>

namespace simulation_interface {

// =============================================================================
class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON(unsigned int axle) const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual chrono::ChContactMethod ContactMethod() const = 0;
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
    virtual chrono::ChContactMethod ContactMethod() const override { return chrono::ChContactMethod::SMC; }
};

namespace Input {
enum {
  STEERING,
  THROTTLE,
  BRAKE,
  TERRAIN_HEIGHT_FL,
  TERRAIN_HEIGHT_FR,
  TERRAIN_HEIGHT_RL,
  TERRAIN_HEIGHT_RR,
  TERRAIN_NORMAL_X_FL,
  TERRAIN_NORMAL_Y_FL,
  TERRAIN_NORMAL_Z_FL,
  TERRAIN_NORMAL_X_FR,
  TERRAIN_NORMAL_Y_FR,
  TERRAIN_NORMAL_Z_FR,
  TERRAIN_NORMAL_X_RL,
  TERRAIN_NORMAL_Y_RL,
  TERRAIN_NORMAL_Z_RL,
  TERRAIN_NORMAL_X_RR,
  TERRAIN_NORMAL_Y_RR,
  TERRAIN_NORMAL_Z_RR,
  TERRAIN_MU_FL,
  TERRAIN_MU_FR,
  TERRAIN_MU_RL,
  TERRAIN_MU_RR,
  LENGTH
};
}

namespace Output {
    enum {
        CHASSIS_POS_X,
        CHASSIS_POS_Y,
        CHASSIS_POS_Z,
        CHASSIS_ORIENT_X,
        CHASSIS_ORIENT_Y,
        CHASSIS_ORIENT_Z,
        CHASSIS_VEL_X,
        CHASSIS_VEL_Y,
        CHASSIS_VEL_Z,
        CHASSIS_ANG_VEL_X,
        CHASSIS_ANG_VEL_Y,
        CHASSIS_ANG_VEL_Z,
        CHASSIS_ACC_X,
        CHASSIS_ACC_Y,
        CHASSIS_ACC_Z,
        CHASSIS_ANG_ACC_X,
        CHASSIS_ANG_ACC_Y,
        CHASSIS_ANG_ACC_Z,
        WHEEL_ANG_VEL_FL,
        WHEEL_ANG_VEL_FR,
        WHEEL_ANG_VEL_RL,
        WHEEL_ANG_VEL_RR,
        TIRE_LONG_SLIP_FL,
        TIRE_LONG_SLIP_FR,
        TIRE_LONG_SLIP_RL,
        TIRE_LONG_SLIP_RR,
        TIRE_LAT_SLIP_FL,
        TIRE_LAT_SLIP_FR,
        TIRE_LAT_SLIP_RL,
        TIRE_LAT_SLIP_RR,
        TIRE_FORCE_LONG_FL,
        TIRE_FORCE_LONG_FR,
        TIRE_FORCE_LONG_RL,
        TIRE_FORCE_LONG_RR,
        TIRE_FORCE_LAT_FL,
        TIRE_FORCE_LAT_FR,
        TIRE_FORCE_LAT_RL,
        TIRE_FORCE_LAT_RR,
        TIRE_FORCE_VERT_FL,
        TIRE_FORCE_VERT_FR,
        TIRE_FORCE_VERT_RL,
        TIRE_FORCE_VERT_RR,
        WHEEL_TORQUE_DRIVE_FL,
        WHEEL_TORQUE_DRIVE_FR,
        WHEEL_TORQUE_DRIVE_RL,
        WHEEL_TORQUE_DRIVE_RR,
        WHEEL_TORQUE_BRAKE_FL,
        WHEEL_TORQUE_BRAKE_FR,
        WHEEL_TORQUE_BRAKE_RL,
        WHEEL_TORQUE_BRAKE_RR,
        STEERING_PINION_ANGLE,
        WHEEL_STEER_ANG_FL,
        WHEEL_STEER_ANG_FR,
        WHEEL_STEER_ANG_RL,
        WHEEL_STEER_ANG_RR,
        QUERY_POINT_X_FL,
        QUERY_POINT_Y_FL,
        QUERY_POINT_Z_FL,
        QUERY_POINT_X_FR,
        QUERY_POINT_Y_FR,
        QUERY_POINT_Z_FR,
        QUERY_POINT_X_RL,
        QUERY_POINT_Y_RL,
        QUERY_POINT_Z_RL,
        QUERY_POINT_X_RR,
        QUERY_POINT_Y_RR,
        QUERY_POINT_Z_RR,
        LENGTH
    };
}

// Terrain that expects input values for terrain properties.
class TerrainInterface : public chrono::vehicle::ChTerrain {
public:
  TerrainInterface(chrono::vehicle::WheeledVehicleForce* vehicle);
  ~TerrainInterface();
  
  // Required implementations of ChTerrain virtual methods
  virtual double GetHeight(const chrono::ChVector3d& loc) const override;
  virtual chrono::ChVector3d GetNormal(const chrono::ChVector3d& loc) const override;
  virtual float GetCoefficientFriction(const chrono::ChVector3d& loc) const override;
  
  // Methods to update terrain properties from external inputs
  void SetTerrainHeight(int wheel_idx, double height);
  void SetTerrainNormal(int wheel_idx, double x, double y, double z);
  void SetTerrainFriction(int wheel_idx, double mu);

  // Store query point information
  void RecordQueryPoint(int wheel_idx, const chrono::ChVector3d& point);
  chrono::ChVector3d GetQueryPoint(int wheel_idx) const;
  
  // Synchronize and Advance methods required by ChTerrain
  virtual void Synchronize(double time) override {}
  virtual void Advance(double step) override {}
  
private:
  // chrono::ChSystem* m_system;
  chrono::vehicle::WheeledVehicleForce* m_vehicle;
  
  // Terrain properties for each wheel
  double m_height[4];
  chrono::ChVector3d m_normal[4];
  double m_friction[4];
  
  // Last query points for each wheel
  // chrono::ChVector3d m_query_point[4];
  
  // Find closest wheel to the specified location
  int FindClosestWheel(const chrono::ChVector3d& loc) const;
};

class CH_VEHICLE_API SimulationInterface {
 public:
  SimulationInterface(const char* vehicle_model_name);
  ~SimulationInterface();
  void Step(const double input[Input::LENGTH], double output[Output::LENGTH]);

  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> SimulationInterface::GetVis() {
    return vis_;
  }

  void SimulationInterface::SetVis(std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis) {
    vis_ = vis;
    vis_->AttachVehicle(car_);
  }

  std::shared_ptr<chrono::vehicle::ChInteractiveDriverIRR> SimulationInterface::GetDriver() {
    return driver_;
  }

  void SimulationInterface::SetDriver(std::shared_ptr<chrono::vehicle::ChInteractiveDriverIRR> driver) {
    driver_ = driver;
  }

 private:
  const double step_size_ = 2e-3;
  chrono::vehicle::WheeledVehicleForce* car_ = nullptr;
  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis_ = nullptr;
  std::shared_ptr<chrono::vehicle::ChInteractiveDriverIRR> driver_ = nullptr;
  std::shared_ptr<TerrainInterface> terrain_ = nullptr;
  // std::shared_ptr<chrono::vehicle::ChTerrain> terrain_ = nullptr;
  Vehicle_Model* vehicle_model_ = nullptr;

  // Wheel indices for convenient access
  static constexpr int FL = 0; // Front Left
  static constexpr int FR = 1; // Front Right
  static constexpr int RL = 2; // Rear Left
  static constexpr int RR = 3; // Rear Right
};

// class SimulationInterface {
//  public:
//   SimulationInterface(const char* config_file);

//   void step(const double input[Input::LENGTH], double out[Output::LENGTH]);

//  private:
//   std::string config_file;
// };

} // namespace simulation_interface

#endif