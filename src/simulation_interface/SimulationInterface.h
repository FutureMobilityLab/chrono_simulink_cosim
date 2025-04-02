#ifndef SIMULATIONINTERFACE_H
#define SIMULATIONINTERFACE_H

#include "chrono/physics/ChContactMaterial.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
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
  SUM,
  DIFF,
  LENGTH
};
}

class Simulation_Interface {
 public:
  Simulation_Interface(const char* vehicle_model_name);
  ~Simulation_Interface();
  void Step(const double input[Input::LENGTH], double output[Output::LENGTH]);
  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> get_vis();

 private:
  const double step_size = 2e-3;
  const double tire_step_size = 1e-3;
  chrono::vehicle::WheeledVehicleForce* car = nullptr;
  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis;
  std::shared_ptr<chrono::vehicle::ChInteractiveDriverIRR> driver;
  chrono::vehicle::RigidTerrain* terrain = nullptr;
  Vehicle_Model* vehicle_model = nullptr;
};

// class Simulation_Interface {
//  public:
//   Simulation_Interface(const char* config_file);

//   void step(const double input[Input::LENGTH], double out[Output::LENGTH]);

//  private:
//   std::string config_file;
// };

} // namespace simulation_interface

#endif