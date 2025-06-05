#include <array>
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <algorithm>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

#include "src/vehicle_fmu/VehicleFMU.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle_fmu;

const std::string VEHICLE_FMU_NAME = "Vehicle_FMU";

// Unpack directory for the FMU
const std::string VEHICLE_UNPACK_DIR = "./tmp_unpack_vehicle_fmu/";

// Demo configuration
struct DemoConfig {
    // Vehicle configuration
    std::string vehicle_config_file = "src/vehicle_fmu/vehicle_config.json";
    
    // Simulation parameters
    double step_size = 1e-3;
    double start_time = 0.0;
    double stop_time = 20.0;
    bool visible = true;
    double fps = 60.0;
    
    // Driver inputs (constant for this demo)
    double steering = 0.0;     // -1.0 to 1.0
    double throttle = 0.5;     // 0.0 to 1.0
    double braking = 0.0;      // 0.0 to 1.0
    
    // Output settings
    std::string output_dir = "./VEHICLE_FMU_OUT";
};

// Class for a dummy wheel that can be used to interact with the FMU's wheels
class DummyWheel : public ChWheel {
  public:
    DummyWheel() : ChWheel("tire_wheel"), m_inertia(ChVector3d(0)) {}
    virtual double GetWheelMass() const override { return 0; }
    virtual const ChVector3d& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }

  private:
    ChVector3d m_inertia;
};

// Structure to keep track of wheel and tire data
struct WheelTire {
    std::string id;
    std::shared_ptr<ChWheel> wheel;
    std::shared_ptr<ChTire> tire;
};

// Create tires for the simulation
void CreateTires(ChSystem& sys, std::array<WheelTire, 4>& wt, const std::string& tire_JSON_front, const std::string& tire_JSON_rear) {
    std::string id[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

    for (int i = 0; i < 4; i++) {
        wt[i].id = id[i];
        auto spindle = chrono_types::make_shared<ChBody>();
        sys.AddBody(spindle);

        wt[i].wheel = chrono_types::make_shared<DummyWheel>();
        wt[i].wheel->Initialize(nullptr, spindle, LEFT);

        // Use front tire model for the first axle, rear tire model for the second
        const std::string& tire_json = (i < 2) ? tire_JSON_front : tire_JSON_rear;
        wt[i].tire = ReadTireJSON(tire_json);
        wt[i].wheel->SetTire(wt[i].tire);
        wt[i].tire->Initialize(wt[i].wheel);
    }
}

// Synchronize tires with the FMU
void SynchronizeTires(double time, 
                     chrono::FmuChronoUnit& vehicle_fmu, 
                     ChTerrain& terrain, 
                     std::array<WheelTire, 4>& wt) {
    for (int i = 0; i < 4; i++) {
        // Get wheel state from vehicle FMU
        WheelState state;
        vehicle_fmu.GetVecVariable(wt[i].id + ".pos", state.pos);
        vehicle_fmu.GetQuatVariable(wt[i].id + ".rot", state.rot);
        vehicle_fmu.GetVecVariable(wt[i].id + ".lin_vel", state.lin_vel);
        vehicle_fmu.GetVecVariable(wt[i].id + ".ang_vel", state.ang_vel);

        // Get tire force
        auto force = wt[i].tire->ReportTireForce(&terrain);

        // Set spindle/wheel state and synchronize tire
        auto spindle = wt[i].wheel->GetSpindle();
        spindle->SetPos(state.pos);
        spindle->SetRot(state.rot);
        spindle->SetLinVel(state.lin_vel);
        spindle->SetAngVelParent(state.ang_vel);
        wt[i].tire->Synchronize(time, terrain);

        // Set tire force to vehicle FMU
        vehicle_fmu.SetVecVariable(wt[i].id + ".point", force.point);
        vehicle_fmu.SetVecVariable(wt[i].id + ".force", force.force);
        vehicle_fmu.SetVecVariable(wt[i].id + ".moment", force.moment);
    }
}

// Advance tires by the specified time step
void AdvanceTires(double step_size, std::array<WheelTire, 4>& wt) {
    for (int i = 0; i < 4; i++) {
        wt[i].tire->Advance(step_size);
    }
}

// Main function
int main(int argc, char* argv[]) {
    // Create demo configuration - use defaults or read from command line
    DemoConfig config;
    
    // Parse command line arguments
    if (argc > 1) {
        config.vehicle_config_file = argv[1];
    }
    if (argc > 2) {
        config.stop_time = std::stod(argv[2]);
    }
    if (argc > 3) {
        config.throttle = std::stod(argv[3]);
    }
    
    // Load vehicle configuration
    FMU_VehicleModelConfig vehicle_config(config.vehicle_config_file);
    std::cout << "Vehicle configuration loaded from: " << config.vehicle_config_file << std::endl;
    
    // Create output directory if it doesn't exist
    if (!filesystem::create_directory(filesystem::path(config.output_dir))) {
        std::cout << "Output directory " << config.output_dir << " already exists or couldn't be created." << std::endl;
    }
    
    // Debug logging categories for FMU
    std::vector<std::string> logCategories = {"logAll"};
    
    // Initialize FMU
    FmuChronoUnit vehicle_fmu;
    try {
        // TODO: When FMU creation is implemented, we can generate the FMU here
        // For now, we can only work with pre-existing FMUs 
        // CreateVehicleFMU(config.vehicle_config_file, "./", VEHICLE_FMU_NAME);
        
        // The following would load an FMU that was previously created
        // vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, VEHICLE_FMU_FILENAME, VEHICLE_UNPACK_DIR);
        
        // For this demo, we'll just print a message indicating this would be done
        std::cout << "In a fully implemented system, the FMU would be created and loaded here." << std::endl;
        std::cout << "For now, we'll simulate the co-simulation process without an actual FMU." << std::endl;
    }
    catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << std::endl;
        return 1;
    }
    
    // Create terrain
    ChSystemSMC terrain_system;
    FlatTerrain terrain(0.0, 0.8);
    
    // Create wheels and tires
    ChSystemSMC tire_system;
    std::array<WheelTire, 4> wt;
    CreateTires(tire_system, wt, 
                vehicle::GetDataFile(vehicle_config.tire_JSON_front), 
                vehicle::GetDataFile(vehicle_config.tire_JSON_rear));
    
    // Initialize CSV output
    utils::ChWriterCSV csv;
    csv.SetDelimiter(" ");
    
    // SIMULATION LOOP (normally would use actual FMU, here we simulate the process)
    double time = 0;
    double chassis_pos_x = 0, chassis_pos_y = 0, chassis_pos_z = 0.5;
    double chassis_vel_x = 0;
    
    ChTimer<> timer;
    timer.start();
    
    std::cout << "Starting simulation..." << std::endl;
    
    while (time < config.stop_time) {
        // Update the simulation time display every second
        if (fmod(time, 1.0) < config.step_size)
            std::cout << "Time: " << time << "s" << std::endl;
        
        // Set driver inputs (constant for this demo)
        // In a real implementation, these would be set in the FMU:
        //   vehicle_fmu.SetVariable("steering", config.steering, FmuVariable::Type::Real);
        //   vehicle_fmu.SetVariable("throttle", config.throttle, FmuVariable::Type::Real);
        //   vehicle_fmu.SetVariable("braking", config.braking, FmuVariable::Type::Real);
        
        // In the demo simulation, we'll use simple physics to estimate position
        // This is just for demonstration - a real FMU would contain the full vehicle model
        chassis_vel_x += config.throttle * 0.1 * config.step_size;  // Simple acceleration model
        chassis_vel_x *= 0.99;  // Simple drag
        
        chassis_pos_x += chassis_vel_x * config.step_size;
        chassis_pos_y += config.steering * 0.1 * chassis_vel_x * config.step_size;  // Simple steering model
        
        // Save output data
        csv << time << chassis_pos_x << chassis_pos_y << chassis_pos_z << chassis_vel_x << std::endl;
        
        // Advance time
        time += config.step_size;
        
        // In a real implementation with an FMU, we would:
        // 1. Exchange data between FMU and tires:
        //    SynchronizeTires(time, vehicle_fmu, terrain, wt);
        
        // 2. Advance FMU:
        //    vehicle_fmu.DoStep(time, config.step_size, fmi2True);
        
        // 3. Advance tires:
        //    AdvanceTires(config.step_size, wt);
        //    tire_system.DoStepDynamics(config.step_size);
    }
    
    timer.stop();
    std::cout << "Simulation complete." << std::endl;
    std::cout << "Sim time: " << time << " seconds" << std::endl;
    std::cout << "Run time: " << timer() << " seconds" << std::endl;
    
    // Write output data
    std::string output_file = config.output_dir + "/vehicle_trajectory.dat";
    csv.WriteToFile(output_file);
    std::cout << "Output written to: " << output_file << std::endl;
    
    return 0;
} 