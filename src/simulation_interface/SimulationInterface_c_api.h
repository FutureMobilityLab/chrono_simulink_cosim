#ifndef SIMULATIONINTERFACE_C_API_H
#define SIMULATIONINTERFACE_C_API_H

#include "src/simulation_interface/SimulationInterface.h"
#include <cstddef>

// Enum for C API error codes
typedef enum {
    SIM_API_OK = 0,                 // Success
    SIM_API_ERROR_NULL_POINTER,     // An input object pointer was null
    SIM_API_ERROR_INVALID_ARGUMENT, // Invalid argument provided (e.g., bad config_file)
    SIM_API_ERROR_INTERNAL_FAILURE, // General internal C++ error (e.g., exception caught)
    // Add other specific error codes as needed
} SimApiErrorCode;

// C-style wrapper functions to interface with Julia and Python
extern "C" {
    // Create a new Simulation_Interface object
    // Returns an error code, passes the created object pointer via an output parameter.
    CH_VEHICLE_API int CreateSimulationInterface(const char* config_file, simulation_interface::SimulationInterface** obj_out);

    // Get the fixed step size
    // Returns an error code, passes the step size via an output parameter.
    CH_VEHICLE_API int GetStepSize(simulation_interface::SimulationInterface* obj, double* step_size_out);
    
    // Delete a Simulation_Interface object
    // Typically void for destructors, but can return int for error handling if obj is nullptr.
    // Keeping as void for simplicity as common practice for delete functions.
    CH_VEHICLE_API int DestroySimulationInterface(simulation_interface::SimulationInterface* obj);
    
    // Call the step method
    // Returns an error code.
    CH_VEHICLE_API int Step(
        simulation_interface::SimulationInterface* obj,
        const double* input,
        std::size_t input_len,
        double* output,
        std::size_t output_len
    );

    // Add other existing getters like GetSimulationTime, GetChronoSystemPointer, etc.
    // These should also be modified to return int, passing their value via output parameters.
    CH_VEHICLE_API int GetSimulationTime(simulation_interface::SimulationInterface* obj, double* time_out);
    CH_VEHICLE_API int GetChronoInternalFixedStepSize(simulation_interface::SimulationInterface* obj, double* step_size_out);

    // Set data paths
    CH_VEHICLE_API int SetChronoDataPath(const char* path);
    CH_VEHICLE_API int SetVehicleDataPath(const char* path);
}

#endif // SIMULATIONINTERFACE_C_API_H