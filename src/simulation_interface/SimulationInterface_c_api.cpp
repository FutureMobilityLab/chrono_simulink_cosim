#include "src/simulation_interface/SimulationInterface_c_api.h"

extern "C" {
  EXPORT_API simulation_interface::Simulation_Interface* CreateSimulationInterface(const char* config_file) {
    return new simulation_interface::Simulation_Interface(config_file);
  }

  EXPORT_API void DestroySimulationInterface(simulation_interface::Simulation_Interface* obj) {
    delete obj;
  }

  EXPORT_API void Step(
    simulation_interface::Simulation_Interface* obj, 
    const double input[simulation_interface::Input::LENGTH], 
    double output[simulation_interface::Output::LENGTH]
  ) {
    obj->Step(input, output);
  }
} 