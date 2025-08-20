# simulation_interface_py_wrapper.py

import ctypes
import os
import sys
import numpy as np
import numpy.typing as npt
import pandas as pd
import matplotlib.pyplot as plt
from typing import Union


C_API_DLL_PATH = r"C:\Users\15309\Project_Chrono\chrono_simulink_cosim\build\lib\Release\simulation_interface_c_api.dll"

# Add the directory containing the DLL to PATH or sys.path
DLL_DIR = os.path.dirname(C_API_DLL_PATH)
if DLL_DIR not in os.environ["PATH"]:
    os.environ["PATH"] = DLL_DIR + os.pathsep + os.environ["PATH"]
    print(f"Temporarily added '{DLL_DIR}' to PATH for DLL discovery.")


class SimApiErrorCode:
    """Error codes returned by the C API functions."""

    OK = 0
    ERROR_NULL_POINTER = 1
    ERROR_INVALID_ARGUMENT = 2
    ERROR_INTERNAL_FAILURE = 3


class InterfaceInput:
    """Constants for indexing the simulation input array, matching C++ enum."""

    STEERING = 0
    THROTTLE = 1
    BRAKE = 2
    TERRAIN_HEIGHT_FL = 3
    TERRAIN_HEIGHT_FR = 4
    TERRAIN_HEIGHT_RL = 5
    TERRAIN_HEIGHT_RR = 6
    TERRAIN_NORMAL_X_FL = 7
    TERRAIN_NORMAL_Y_FL = 8
    TERRAIN_NORMAL_Z_FL = 9
    TERRAIN_NORMAL_X_FR = 10
    TERRAIN_NORMAL_Y_FR = 11
    TERRAIN_NORMAL_Z_FR = 12
    TERRAIN_NORMAL_X_RL = 13
    TERRAIN_NORMAL_Y_RL = 14
    TERRAIN_NORMAL_Z_RL = 15
    TERRAIN_NORMAL_X_RR = 16
    TERRAIN_NORMAL_Y_RR = 17
    TERRAIN_NORMAL_Z_RR = 18
    TERRAIN_MU_FL = 19
    TERRAIN_MU_FR = 20
    TERRAIN_MU_RL = 21
    TERRAIN_MU_RR = 22
    LENGTH = 23


class InterfaceOutput:
    """Constants for indexing the simulation output array, matching C++ enum."""

    CHASSIS_POS_X = 0
    CHASSIS_POS_Y = 1
    CHASSIS_POS_Z = 2
    CHASSIS_ORIENT_X = 3
    CHASSIS_ORIENT_Y = 4
    CHASSIS_ORIENT_Z = 5
    CHASSIS_VEL_X = 6
    CHASSIS_VEL_Y = 7
    CHASSIS_VEL_Z = 8
    CHASSIS_ANG_VEL_X = 9
    CHASSIS_ANG_VEL_Y = 10
    CHASSIS_ANG_VEL_Z = 11
    CHASSIS_ACC_X = 12
    CHASSIS_ACC_Y = 13
    CHASSIS_ACC_Z = 14
    CHASSIS_ANG_ACC_X = 15
    CHASSIS_ANG_ACC_Y = 16
    CHASSIS_ANG_ACC_Z = 17
    WHEEL_ANG_VEL_FL = 18
    WHEEL_ANG_VEL_FR = 19
    WHEEL_ANG_VEL_RL = 20
    WHEEL_ANG_VEL_RR = 21
    TIRE_LONG_SLIP_FL = 22
    TIRE_LONG_SLIP_FR = 23
    TIRE_LONG_SLIP_RL = 24
    TIRE_LONG_SLIP_RR = 25
    TIRE_LAT_SLIP_FL = 26
    TIRE_LAT_SLIP_FR = 27
    TIRE_LAT_SLIP_RL = 28
    TIRE_LAT_SLIP_RR = 29
    TIRE_FORCE_LONG_FL = 30
    TIRE_FORCE_LONG_FR = 31
    TIRE_FORCE_LONG_RL = 32
    TIRE_FORCE_LONG_RR = 33
    TIRE_FORCE_LAT_FL = 34
    TIRE_FORCE_LAT_FR = 35
    TIRE_FORCE_LAT_RL = 36
    TIRE_FORCE_LAT_RR = 37
    TIRE_FORCE_VERT_FL = 38
    TIRE_FORCE_VERT_FR = 39
    TIRE_FORCE_VERT_RL = 40
    TIRE_FORCE_VERT_RR = 41
    WHEEL_TORQUE_DRIVE_FL = 42
    WHEEL_TORQUE_DRIVE_FR = 43
    WHEEL_TORQUE_DRIVE_RL = 44
    WHEEL_TORQUE_DRIVE_RR = 45
    WHEEL_TORQUE_BRAKE_FL = 46
    WHEEL_TORQUE_BRAKE_FR = 47
    WHEEL_TORQUE_BRAKE_RL = 48
    WHEEL_TORQUE_BRAKE_RR = 49
    STEERING_PINION_ANGLE = 50
    WHEEL_STEER_ANG_FL = 51
    WHEEL_STEER_ANG_FR = 52
    WHEEL_STEER_ANG_RL = 53
    WHEEL_STEER_ANG_RR = 54
    QUERY_POINT_X_FL = 55
    QUERY_POINT_Y_FL = 56
    QUERY_POINT_Z_FL = 57
    QUERY_POINT_X_FR = 58
    QUERY_POINT_Y_FR = 59
    QUERY_POINT_Z_FR = 60
    QUERY_POINT_X_RL = 61
    QUERY_POINT_Y_RL = 62
    QUERY_POINT_Z_RL = 63
    QUERY_POINT_X_RR = 64
    QUERY_POINT_Y_RR = 65
    QUERY_POINT_Z_RR = 66
    SIM_TIME = 67
    LENGTH = 68


# --- Define C types for array inputs/outputs using the correct lengths ---
C_DOUBLE_ARRAY_INPUT = ctypes.c_double * InterfaceInput.LENGTH
C_DOUBLE_ARRAY_OUTPUT = ctypes.c_double * InterfaceOutput.LENGTH


try:
    # Load the shared library
    _sim_lib = ctypes.CDLL(C_API_DLL_PATH)
    print(f"Successfully loaded C API library from: {C_API_DLL_PATH}")
except OSError as e:
    print(f"Error loading C API library: {e}", file=sys.stderr)
    print(
        f"Please ensure the DLL/SO '{os.path.basename(C_API_DLL_PATH)}' exists at '{C_API_DLL_PATH}'.",
        file=sys.stderr,
    )
    print(
        "Also, check if all its *transitive* dependencies (e.g., Chrono DLLs, Irrlicht.dll) are in the same directory or discoverable via system PATH.",
        file=sys.stderr,
    )
    sys.exit(1)

# CreateSimulationInterface(const char* config_file, simulation_interface::SimulationInterface** obj_out)
_sim_lib.CreateSimulationInterface.argtypes = [
    ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_void_p),
]
_sim_lib.CreateSimulationInterface.restype = ctypes.c_int  # Returns error code

# GetStepSize(simulation_interface::SimulationInterface* obj, double* step_size_out)
_sim_lib.GetStepSize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
_sim_lib.GetStepSize.restype = ctypes.c_int  # Returns error code

# DestroySimulationInterface(simulation_interface::SimulationInterface* obj)
_sim_lib.DestroySimulationInterface.argtypes = [ctypes.c_void_p]
_sim_lib.DestroySimulationInterface.restype = ctypes.c_int  # Returns error code

# Step(simulation_interface::SimulationInterface* obj, const double input[], double output[])
_sim_lib.Step.argtypes = [
    ctypes.c_void_p,
    C_DOUBLE_ARRAY_INPUT,
    C_DOUBLE_ARRAY_OUTPUT,
]
_sim_lib.Step.restype = ctypes.c_int  # Returns error code

# GetSimulationTime(simulation_interface::SimulationInterface* obj, double* time_out)
_sim_lib.GetSimulationTime.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
_sim_lib.GetSimulationTime.restype = ctypes.c_int  # Returns error code


class SimulationInterface:
    """
    Python wrapper for the C++ SimulationInterface class via its C API.
    Handles input/output using pandas DataFrames.
    """

    def __init__(self, vehicle_type: str):
        """
        Initializes the SimulationInterface object.

        Args:
            vehicle_type (str): Path to the configuration file.
        """
        # Convert vehicle_type string to bytes for C char*
        config_file_bytes = vehicle_type.encode("utf-8")

        # Prepare a ctypes pointer to hold the C++ object pointer
        obj_ptr_out = ctypes.c_void_p()

        # Call the C API and check the error code
        result_code = _sim_lib.CreateSimulationInterface(
            config_file_bytes, ctypes.byref(obj_ptr_out)
        )

        if result_code != SimApiErrorCode.OK:
            raise RuntimeError(
                f"Failed to create C++ SimulationInterface object. "
                f"C API returned error code: {result_code}."
            )

        # Store the pointer from the output parameter
        self._obj_ptr = obj_ptr_out

        print(f"SimulationInterface created with config: {vehicle_type}")

    def __del__(self):
        """
        Destroys the underlying C++ SimulationInterface object when the Python object is garbage collected.
        """
        if hasattr(self, "_obj_ptr") and self._obj_ptr:
            _sim_lib.DestroySimulationInterface(self._obj_ptr)
            self._obj_ptr = None

    def get_step_size(self):
        """
        Calls the C API to retrieve the simulation step size.
        Returns:
            float: The simulation step size.
        """
        step_size_out = ctypes.c_double()
        result_code = _sim_lib.GetStepSize(self._obj_ptr, ctypes.byref(step_size_out))

        if result_code != SimApiErrorCode.OK:
            raise RuntimeError(
                f"Failed to get step size from C API. " f"Error code: {result_code}."
            )

        return step_size_out.value

    def step(self, input: Union[pd.DataFrame, pd.Series, npt.NDArray]) -> npt.NDArray:
        """
        Performs a simulation step.

        Args:
            input (Union[pd.DataFrame, pd.Series, npt.NDArray]):
                A pandas DataFrame, Series, or NumPy array.

        Returns:
            pd.DataFrame: A pandas DataFrame with a single row,
                          its columns corresponding to the InterfaceOutput constants.
        """
        if isinstance(input, pd.DataFrame) or isinstance(input, pd.Series):
            input_np = input.to_numpy(dtype=np.float64)
        elif isinstance(input, np.ndarray):
            input_np = input
        else:
            raise ValueError(
                "Input must be either DataFrame, Series, or NDArray "
                f"got {type(input)}"
            )

        if input_np.ndim == 2:
            n_rows, n_cols = input_np.shape
            if n_rows > 1 and n_cols > 1:
                raise ValueError(f"Input must be 1D-like, got ({n_rows}, {n_cols})")
            input_np = input_np.flatten()

        if len(input_np) != InterfaceInput.LENGTH:
            raise ValueError(
                f"Input expected to be {InterfaceInput.LENGTH}, but got"
                f"{len(input_np)}"
            )

        # Convert NumPy array to ctypes array
        c_input = C_DOUBLE_ARRAY_INPUT(*input_np)

        # Create a C array for output (will be filled by the C function)
        c_output = C_DOUBLE_ARRAY_OUTPUT()

        # Call the C API step function and check the error code
        result_code = _sim_lib.Step(self._obj_ptr, c_input, c_output)
        if result_code != SimApiErrorCode.OK:
            raise RuntimeError(f"Step failed with C API error code: {result_code}")

        # Convert C output array back to a NumPy array.
        return np.array(list(c_output), dtype=np.float64)


if __name__ == "__main__":
    # Ensure to replace "your_config_file.json" with an actual path
    # relevant to your SimulationInterface's constructor.
    config_file_path = "sedan"  # Example path, replace with your actual config file

    # Create an instance of the wrapper
    sim = SimulationInterface(config_file_path)

    # Example input data as a pandas DataFrame
    # Initialize a DataFrame with zeros and correct columns
    n_steps = int(30 / sim.get_step_size())
    input_data_np = np.zeros((n_steps, InterfaceInput.LENGTH))
    output_data_np = np.zeros((n_steps, InterfaceOutput.LENGTH))

    for i in range(input_data_np.shape[0]):
        input_data_np[i, InterfaceInput.STEERING] = 0.1
        input_data_np[i, InterfaceInput.THROTTLE] = 0.5
        input_data_np[i, InterfaceInput.BRAKE] = 0.0
        input_data_np[i, InterfaceInput.TERRAIN_NORMAL_Z_FL] = 1.0
        input_data_np[i, InterfaceInput.TERRAIN_NORMAL_Z_FR] = 1.0
        input_data_np[i, InterfaceInput.TERRAIN_NORMAL_Z_RL] = 1.0
        input_data_np[i, InterfaceInput.TERRAIN_NORMAL_Z_RR] = 1.0
        input_data_np[i, InterfaceInput.TERRAIN_MU_FL] = 0.8
        input_data_np[i, InterfaceInput.TERRAIN_MU_FR] = 0.8
        input_data_np[i, InterfaceInput.TERRAIN_MU_RL] = 0.8
        input_data_np[i, InterfaceInput.TERRAIN_MU_RR] = 0.8
        output_data_np[i, :] = sim.step(input_data_np[i, :])

    for i in range(output_data_np.shape[1]):
        print(
            f"output_data_np[:, {i}]: [{np.min(output_data_np[:,i]):.6f} -> "
            f"{np.max(output_data_np[:,i]):.6f}]"
        )

    plt.figure()
    plt.plot(
        output_data_np[:, InterfaceOutput.CHASSIS_POS_X],
        output_data_np[:, InterfaceOutput.CHASSIS_POS_Y],
    )
    plt.show()
