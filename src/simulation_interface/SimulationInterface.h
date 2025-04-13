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
        LENGTH
    };
}

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
  chrono::vehicle::RigidTerrain* terrain_ = nullptr;
  Vehicle_Model* vehicle_model_ = nullptr;
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