# Vehicle FMU Library

This library provides tools for creating Functional Mock-up Units (FMUs) from Chrono vehicle models. It serves as a bridge between Chrono's vehicle modeling capabilities and the FMI standard for co-simulation.

## Features

- Create FMUs from Chrono vehicle models defined by JSON files
- Configure vehicles, tires, powertrains, and simulation parameters through a single JSON configuration file
- Co-simulate vehicle dynamics with external systems (tire models, driver models, etc.)
- Extract detailed vehicle state information for analysis

## Requirements

- Project Chrono with the following modules:
  - Vehicle module
  - FMI module
  - [Optional] Irrlicht module (for visualization)

## Using Vehicle FMU

### Creating a Vehicle Configuration

Create a JSON configuration file that specifies the vehicle model and simulation parameters:

```json
{
    "Vehicle": "path/to/vehicle.json",
    "TireFront": "path/to/tire_front.json",
    "TireRear": "path/to/tire_rear.json",
    "Engine": "path/to/engine.json",
    "Transmission": "path/to/transmission.json",
    
    "UseSMC": true,
    "StepSize": 0.002,
    
    "InitialLocation": [0.0, 0.0, 0.5],
    "InitialYaw": 0.0,
    "Gravity": [0.0, 0.0, -9.81],
    
    "Visualization": true,
    "SaveImages": false,
    "FPS": 60.0,
    "OutputDir": "./output",
    "CameraDistance": 6.0
}
```

### Creating an FMU

To create an FMU from your vehicle configuration:

```cpp
#include "src/vehicle_fmu/VehicleFMU.h"

using namespace chrono::vehicle_fmu;

// Create an FMU
bool success = CreateVehicleFMU("path/to/config.json", "output_directory", "MyVehicleFMU");
```

### Running the Demo

The `demo_cosim_fmu` application demonstrates how to use a vehicle FMU with a flat terrain and constant driver inputs:

```bash
./demo_cosim_fmu [config_file] [simulation_time] [throttle]
```

Parameters:
- `config_file`: Path to the vehicle configuration JSON file (default: `src/vehicle_fmu/vehicle_config.json`)
- `simulation_time`: Duration of the simulation in seconds (default: 20.0)
- `throttle`: Constant throttle input between 0.0 and 1.0 (default: 0.5)

## FMU Inputs/Outputs

### Inputs
- `steering`: Steering input [-1.0 to 1.0]
- `throttle`: Throttle input [0.0 to 1.0]
- `braking`: Braking input [0.0 to 1.0]
- `clutch`: Clutch input [0.0 to 1.0]
- `wheel_XX.point`: Tire forces application point
- `wheel_XX.force`: Tire forces
- `wheel_XX.moment`: Tire moments

### Outputs
- `ref_frame`: Vehicle reference frame
- `chassis_pos_x/y/z`: Chassis position
- `chassis_orient_x/y/z`: Chassis orientation
- `chassis_vel_x/y/z`: Chassis velocity
- `chassis_ang_vel_x/y/z`: Chassis angular velocity
- `wheel_XX.pos`: Wheel position
- `wheel_XX.rot`: Wheel rotation
- `wheel_XX.lin_vel`: Wheel linear velocity
- `wheel_XX.ang_vel`: Wheel angular velocity

## Notes

- This implementation is adapted from the existing FMU implementations in Chrono, particularly the `FMU_WheeledVehicle` class
- The current implementation focuses on co-simulation (CS) mode in FMI 2.0
- For full FMU generation capabilities, the Chrono FMI module must be properly configured 