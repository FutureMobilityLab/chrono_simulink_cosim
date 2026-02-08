#include "src/simulation_interface/SimulationInterface_c_api.h" // Includes SimApiErrorCode enum
#include <iostream> // For std::cerr
#include <cstring>  // For std::strlen
#include <stdexcept> // For std::invalid_argument, std::exception

extern "C" {

// Create a new Simulation_Interface object
// Returns an error code, and passes the created object pointer via an output parameter.
CH_VEHICLE_API int CreateSimulationInterface(const char* config_file, simulation_interface::SimulationInterface** obj_out) {
  if (obj_out == nullptr) {
    std::cerr << "C API Error (CreateSimulationInterface): Output pointer 'obj_out' is null." << std::endl;
    return SIM_API_ERROR_NULL_POINTER;
  }
  *obj_out = nullptr; // Initialize output pointer to null to avoid dangling pointers on error

  if (config_file == nullptr || std::strlen(config_file) == 0) {
    std::cerr << "C API Error (CreateSimulationInterface): Configuration file path is null or empty." << std::endl;
    return SIM_API_ERROR_INVALID_ARGUMENT;
  }

  try {
    *obj_out = new simulation_interface::SimulationInterface(config_file);
    return SIM_API_OK;
  } catch (const std::invalid_argument& e) {
    std::cerr << "C API Error (CreateSimulationInterface): Invalid argument during creation: " << e.what() << std::endl;
    return SIM_API_ERROR_INVALID_ARGUMENT;
  } catch (const std::exception& e) {
    std::cerr << "C API Error (CreateSimulationInterface): General C++ exception during creation: " << e.what() << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  } catch (...) {
    std::cerr << "C API Error (CreateSimulationInterface): Unknown C++ exception during creation." << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  }
}

// Get the fixed step size
// Returns an error code, and passes the step size via an output parameter.
// Assuming SimulationInterface has a static method or global constant GetDefaultStepSize()
CH_VEHICLE_API int GetStepSize(simulation_interface::SimulationInterface* obj, double* step_size_out) {
  if (obj == nullptr) {
    std::cerr << "C API Error (GetStepSize): Received null SimulationInterface object." << std::endl;
    return SIM_API_ERROR_NULL_POINTER; // Return specific error code
  }
  if (step_size_out == nullptr) {
    std::cerr << "C API Error (GetStepSize): Output pointer 'step_size_out' is null." << std::endl;
    return SIM_API_ERROR_NULL_POINTER;
  }
  try {
    *step_size_out = obj->GetStepSize();
    return SIM_API_OK;
  } catch (const std::exception& e) {
    std::cerr << "C API Error (GetStepSize): C++ exception caught: " << e.what() << std::endl;
    *step_size_out = -1.0; // Set an invalid default
    return SIM_API_ERROR_INTERNAL_FAILURE;
  } catch (...) {
    std::cerr << "C API Error (GetStepSize): Unknown C++ exception caught." << std::endl;
    *step_size_out = -1.0;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  }
}

// Delete a Simulation_Interface object
// Returns an error code for robustness, though `delete nullptr` is safe.
CH_VEHICLE_API int DestroySimulationInterface(simulation_interface::SimulationInterface* obj) {
  if (obj == nullptr) {
    // Deleting nullptr is safe, so this might just be for logging/consistency
    std::cerr << "C API Warning (DestroySimulationInterface): Received null object to destroy. Doing nothing." << std::endl;
    return SIM_API_OK; // Or SIM_API_ERROR_NULL_POINTER if strict
  }
  try {
    delete obj;
    return SIM_API_OK;
  } catch (const std::exception& e) {
    std::cerr << "C API Error (DestroySimulationInterface): C++ exception during deletion: " << e.what() << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  } catch (...) {
    std::cerr << "C API Error (DestroySimulationInterface): Unknown C++ exception during deletion." << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  }
}

// Call the step method
CH_VEHICLE_API int Step(
    simulation_interface::SimulationInterface* obj,
    const double* input,
    std::size_t input_len,
    double* output,
    std::size_t output_len
) {
  if (obj == nullptr) {
    std::cerr << "C API Error (Step): Received null SimulationInterface object." << std::endl;
    return SIM_API_ERROR_NULL_POINTER; // Return specific error code
  }
  // You might also add checks for input/output arrays being nullptr if they can be
  if (input == nullptr || output == nullptr) {
    std::cerr << "C API Error (Step): Input or output array is null." << std::endl;
    return SIM_API_ERROR_NULL_POINTER;
  }

  // Validate lengths passed by the caller to avoid out-of-bounds reads/writes
  if (input_len != simulation_interface::Input::LENGTH || output_len != simulation_interface::Output::LENGTH) {
    std::cerr << "C API Error (Step): Invalid input/output length. Expected input_len="
              << simulation_interface::Input::LENGTH << ", output_len=" << simulation_interface::Output::LENGTH
              << ", got input_len=" << input_len << ", output_len=" << output_len << std::endl;
    return SIM_API_ERROR_INVALID_ARGUMENT;
  }

  try {
    // Copy the incoming input array into a local buffer. When called from
    // foreign runtimes (Python/ctypes, Julia, etc.) the caller's buffer may
    // have different alignment, layout, or a lifetime shorter than the
    // callee expects. Copying here ensures a stable, properly-aligned C
    // array is passed into the C++ implementation.
    double local_input[simulation_interface::Input::LENGTH];
    std::memcpy(local_input, input, sizeof(double) * simulation_interface::Input::LENGTH);

    // Initialize output to a known state to avoid propagating uninitialized
    // garbage in case the implementation reads or validates output before
    // fully writing it.
    for (size_t i = 0; i < simulation_interface::Output::LENGTH; ++i) {
      output[i] = 0.0;
    }

    obj->Step(local_input, output); // Call the actual C++ method with safe copy
    return SIM_API_OK; // Return success code
  } catch (const std::exception& e) {
    std::cerr << "C API Error (Step): C++ exception caught during step: " << e.what() << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE; // Translate C++ exception to a C error code
  } catch (...) {
    std::cerr << "C API Error (Step): Unknown C++ exception caught during step." << std::endl;
    return SIM_API_ERROR_INTERNAL_FAILURE;
  }
}

// Get current simulation time
CH_VEHICLE_API int GetSimulationTime(simulation_interface::SimulationInterface* obj, double* time_out) {
    if (obj == nullptr || time_out == nullptr) {
        std::cerr << "C API Error (GetSimulationTime): Null pointer detected." << std::endl;
        if (time_out) *time_out = -1.0; // Indicate error with a default value
        return SIM_API_ERROR_NULL_POINTER;
    }
    try {
        *time_out = obj->GetSimTime(); // Assuming obj->GetSystemPtr() is valid
        return SIM_API_OK;
    } catch (const std::exception& e) {
        std::cerr << "C API Error (GetSimulationTime): C++ exception caught: " << e.what() << std::endl;
        *time_out = -1.0;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    } catch (...) {
        std::cerr << "C API Error (GetSimulationTime): Unknown C++ exception caught." << std::endl;
        *time_out = -1.0;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    }
}


CH_VEHICLE_API int SetChronoDataPath(const char* path) {
    if (path == nullptr || std::strlen(path) == 0) {
        std::cerr << "C API Error (SetChronoDataPath): Path is null or empty." << std::endl;
        return SIM_API_ERROR_INVALID_ARGUMENT;
    }
    try {
        chrono::SetChronoDataPath(path);
        return SIM_API_OK;
    } catch (const std::exception& e) {
        std::cerr << "C API Error (SetChronoDataPath): C++ exception caught: " << e.what() << std::endl;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    } catch (...) {
        std::cerr << "C API Error (SetChronoDataPath): Unknown C++ exception caught." << std::endl;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    }
}

CH_VEHICLE_API int SetVehicleDataPath(const char* path) {
    if (path == nullptr || std::strlen(path) == 0) {
        std::cerr << "C API Error (SetVehicleDataPath): Path is null or empty." << std::endl;
        return SIM_API_ERROR_INVALID_ARGUMENT;
    }
    try {
        chrono::vehicle::SetDataPath(path);
        return SIM_API_OK;
    } catch (const std::exception& e) {
        std::cerr << "C API Error (SetVehicleDataPath): C++ exception caught: " << e.what() << std::endl;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    } catch (...) {
        std::cerr << "C API Error (SetVehicleDataPath): Unknown C++ exception caught." << std::endl;
        return SIM_API_ERROR_INTERNAL_FAILURE;
    }
}

} // extern "C"