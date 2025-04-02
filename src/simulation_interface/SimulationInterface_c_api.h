#ifndef SIMULATIONINTERFACE_C_API_H
#define SIMULATIONINTERFACE_C_API_H

#include "src/simulation_interface/SimulationInterface.h"

// Define the macro for exporting functions
#ifdef _WIN32
  #ifdef SIMULATION_INTERFACE_EXPORTS
    #define EXPORT_API __declspec(dllexport)
  #else
    #define EXPORT_API __declspec(dllimport)
  #endif
#else
  #define EXPORT_API
#endif

// C-style wrapper functions to interface with Julia
extern "C" {
  // Create a new Simulation_Interface object
  EXPORT_API simulation_interface::Simulation_Interface* CreateSimulationInterface(const char* config_file);
  
  // Delete a Simulation_Interface object
  EXPORT_API void DestroySimulationInterface(simulation_interface::Simulation_Interface* obj);
  
  // Call the step method
  EXPORT_API void Step(
    simulation_interface::Simulation_Interface* obj, 
    const double input[simulation_interface::Input::LENGTH], 
    double output[simulation_interface::Output::LENGTH]
  );
}

#endif // SIMULATIONINTERFACE_C_API_H 