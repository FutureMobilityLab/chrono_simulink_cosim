#ifndef SIMULATIONINTERFACE_C_API_H
#define SIMULATIONINTERFACE_C_API_H

#include "src/simulation_interface/SimulationInterface.h"

// C-style wrapper functions to interface with Julia
extern "C" {
  // Create a new Simulation_Interface object
  CH_VEHICLE_API simulation_interface::SimulationInterface* CreateSimulationInterface(const char* config_file);
  
  // Delete a Simulation_Interface object
  CH_VEHICLE_API void DestroySimulationInterface(simulation_interface::SimulationInterface* obj);
  
  // Call the step method
  CH_VEHICLE_API void Step(
    simulation_interface::SimulationInterface* obj, 
    const double input[simulation_interface::Input::LENGTH], 
    double output[simulation_interface::Output::LENGTH]
  );
}

#endif // SIMULATIONINTERFACE_C_API_H 