#include "src/simulation_interface/SimulationInterface_c_api.h"

extern "C" {
  simulation_interface::SimulationInterface* CreateSimulationInterface(const char* config_file) {
    return new simulation_interface::SimulationInterface(config_file);
  }

  void DestroySimulationInterface(simulation_interface::SimulationInterface* obj) {
    delete obj;
  }

  void Step(
    simulation_interface::SimulationInterface* obj, 
    const double input[simulation_interface::Input::LENGTH], 
    double output[simulation_interface::Output::LENGTH]
  ) {
    obj->Step(input, output);
  }
} 