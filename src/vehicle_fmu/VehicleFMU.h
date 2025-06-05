#ifndef VEHICLEFMU_H
#define VEHICLEFMU_H

#include <string>
#include <memory>
#include <array>
#include <vector>
#include <limits>

#include "chrono/physics/ChContactMaterial.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_fmi/fmi2/ChFmuToolsExport.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle_fmu {

// FMU model configuration class, can be loaded from JSON
class FMU_VehicleModelConfig {
public:
    FMU_VehicleModelConfig() = default;
    FMU_VehicleModelConfig(const std::string& filename);
    
    // Core model configuration
    std::string vehicle_JSON;
    std::string tire_JSON_front;
    std::string tire_JSON_rear;
    std::string engine_JSON;
    std::string transmission_JSON;
    
    // System configuration
    bool use_SMC = true;
    double step_size = 2e-3;
    
    // Initial conditions
    ChVector3d init_loc = {0, 0, 0.5};
    double init_yaw = 0.0;
    ChVector3d g_acc = {0, 0, -9.81};
    
    // Visualization
    bool visualization = true;
    bool save_images = false;
    double fps = 60;
    std::string output_dir = "./";
    double camera_distance = 6.0;

    // Load configuration from JSON
    bool LoadFromJSON(const std::string& filename);
};

// Input/Output definitions
namespace Input {
enum {
    STEERING,
    THROTTLE,
    BRAKE,
    CLUTCH,
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

// Class that creates an FMU component for vehicle simulations
class FmuVehicleComponent : public chrono::FmuChronoComponentBase {
public:
    FmuVehicleComponent(fmi2String instanceName,
                        fmi2Type fmuType,
                        fmi2String fmuGUID,
                        fmi2String fmuResourceLocation,
                        const fmi2CallbackFunctions* functions,
                        fmi2Boolean visible,
                        fmi2Boolean loggingOn);
    
    ~FmuVehicleComponent() {}

    /// Advance dynamics
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

private:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;
    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    // Create the vehicle system
    void CreateVehicle();
    
    // Configure the underlying Chrono system
    void ConfigureSystem();
    
    // Update vehicle system with current FMU continuous inputs
    void SynchronizeVehicle(double time);
    
    // Extract FMU continuous outputs from the vehicle system
    void CalculateVehicleOutputs();

    // Exchange data for vehicle wheels
    struct WheelData {
        std::shared_ptr<chrono::vehicle::ChWheel> wheel;
        std::string identifier;
        chrono::vehicle::WheelState state;
        chrono::vehicle::TerrainForce load;
        
        // Terrain information at wheel location (inputs)
        double terrain_height;
        chrono::ChVector3d terrain_normal;
        double terrain_mu;
    };

    std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle;  ///< underlying wheeled vehicle

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis_sys;
#endif

    // FMU configuration
    FMU_VehicleModelConfig config;
    
    // FMU I/O parameters
    std::string data_path;          ///< path to vehicle data
    
    // FMU continuous inputs and outputs for co-simulation
    chrono::vehicle::DriverInputs driver_inputs;  ///< vehicle control inputs (input)
    std::array<WheelData, 4> wheel_data;          ///< wheel state and applied forces (output/input)
    chrono::ChFrameMoving<> ref_frame;            ///< vehicle reference frame (output)
    
    // Output array for direct use
    double output_values[Output::LENGTH];

    int render_frame;  ///< counter for rendered frames

    // Terrain information class for tire-terrain interaction
    class LocalTerrain : public chrono::vehicle::ChTerrain {
      public:
        LocalTerrain() {}
        virtual double GetHeight(const chrono::ChVector3d& loc) const override;
        virtual chrono::ChVector3d GetNormal(const chrono::ChVector3d& loc) const override;
        virtual float GetCoefficientFriction(const chrono::ChVector3d& loc) const override;
        
        // Wheel-specific terrain data
        std::array<WheelData, 4>* wheel_data;
    };

    std::shared_ptr<LocalTerrain> terrain;  ///< terrain for tire interaction
};

// Function to create an FMU from a vehicle model
bool CH_VEHICLE_API CreateVehicleFMU(const std::string& model_config_file, 
                                    const std::string& output_directory,
                                    const std::string& fmu_name = "Vehicle_FMU");

// Helper function to create a demo FMU instance
FmuChronoComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName,
                                             fmi2Type fmuType,
                                             fmi2String fmuGUID,
                                             fmi2String fmuResourceLocation,
                                             const fmi2CallbackFunctions* functions,
                                             fmi2Boolean visible,
                                             fmi2Boolean loggingOn) {
    return new FmuVehicleComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

} // namespace vehicle_fmu
} // namespace chrono

#endif // VEHICLEFMU_H 