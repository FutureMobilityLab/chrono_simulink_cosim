#include "src/vehicle_fmu/VehicleFMU.h"

#include <cassert>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <ctime>

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle_fmu {

// Implementation of FMU_VehicleModelConfig
FMU_VehicleModelConfig::FMU_VehicleModelConfig(const std::string& filename) {
    LoadFromJSON(filename);
}

bool FMU_VehicleModelConfig::LoadFromJSON(const std::string& filename) {
    try {
        // Use Chrono's utility function to load the JSON file
        rapidjson::Document d;
        vehicle::ReadFileJSON(filename, d);
        
        if (d.IsNull()) {
            std::cerr << "Error loading model configuration file: " << filename << std::endl;
            return false;
        }

        // Parse core model configuration
        if (d.HasMember("Vehicle")) {
            vehicle_JSON = d["Vehicle"].GetString();
        }
        if (d.HasMember("TireFront")) {
            tire_JSON_front = d["TireFront"].GetString();
        }
        if (d.HasMember("TireRear")) {
            tire_JSON_rear = d["TireRear"].GetString();
        }
        if (d.HasMember("Engine")) {
            engine_JSON = d["Engine"].GetString();
        }
        if (d.HasMember("Transmission")) {
            transmission_JSON = d["Transmission"].GetString();
        }

        // Parse system configuration
        if (d.HasMember("UseSMC")) {
            use_SMC = d["UseSMC"].GetBool();
        }
        if (d.HasMember("StepSize")) {
            step_size = d["StepSize"].GetDouble();
        }

        // Parse initial conditions
        if (d.HasMember("InitialLocation")) {
            init_loc = vehicle::ReadVectorJSON(d["InitialLocation"]);
        }
        if (d.HasMember("InitialYaw")) {
            init_yaw = d["InitialYaw"].GetDouble();
        }
        if (d.HasMember("Gravity")) {
            g_acc = vehicle::ReadVectorJSON(d["Gravity"]);
        }

        // Parse visualization options
        if (d.HasMember("Visualization")) {
            visualization = d["Visualization"].GetBool();
        }
        if (d.HasMember("SaveImages")) {
            save_images = d["SaveImages"].GetBool();
        }
        if (d.HasMember("FPS")) {
            fps = d["FPS"].GetDouble();
        }
        if (d.HasMember("OutputDir")) {
            output_dir = d["OutputDir"].GetString();
        }
        if (d.HasMember("CameraDistance")) {
            camera_distance = d["CameraDistance"].GetDouble();
        }

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading model configuration: " << e.what() << std::endl;
        return false;
    }
}

// Implementation of FmuVehicleComponent
FmuVehicleComponent::FmuVehicleComponent(fmi2String instanceName,
                       fmi2Type fmuType,
                       fmi2String fmuGUID,
                       fmi2String fmuResourceLocation,
                       const fmi2CallbackFunctions* functions,
                       fmi2Boolean visible,
                       fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn), render_frame(0) {
    
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    driver_inputs = {0, 0, 0, 0};
    
    // Get resources directory from the FMU path
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    data_path = resources_dir + "/";
    
    // Set default JSON file paths from the FMU resources directory
    config.vehicle_JSON = resources_dir + "/Vehicle.json";
    config.tire_JSON_front = resources_dir + "/TMeasyTire.json";
    config.tire_JSON_rear = resources_dir + "/TMeasyTire.json";
    config.engine_JSON = resources_dir + "/EngineShafts.json";
    config.transmission_JSON = resources_dir + "/AutomaticTransmissionShafts.json";
    
    // Initialize output array
    for (int i = 0; i < Output::LENGTH; i++)
        output_values[i] = 0.0;

    // Set wheel identifier strings
    wheel_data[0].identifier = "FL";
    wheel_data[1].identifier = "FR";
    wheel_data[2].identifier = "RL";
    wheel_data[3].identifier = "RR";
    
    // Initialize wheel terrain values
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].terrain_height = 0.0;
        wheel_data[iw].terrain_normal = ChVector3d(0, 0, 1);
        wheel_data[iw].terrain_mu = 0.8;
    }

#ifdef CHRONO_IRRLICHT
    if (visible && config.visualization)
        vis_sys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
#else
    if (visible && config.visualization)
        std::cout << "The FMU was not built with run-time visualization support. Visualization disabled." << std::endl;
#endif

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&data_path, "data_path", FmuVariable::Type::String, "1", "vehicle data path",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVariable(&config.vehicle_JSON, "vehicle_JSON", FmuVariable::Type::String, "1", "vehicle JSON",                 //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&config.tire_JSON_front, "tire_JSON_front", FmuVariable::Type::String, "1", "front tire JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&config.tire_JSON_rear, "tire_JSON_rear", FmuVariable::Type::String, "1", "rear tire JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&config.engine_JSON, "engine_JSON", FmuVariable::Type::String, "1", "engine JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&config.transmission_JSON, "transmission_JSON", FmuVariable::Type::String, "1", "transmission JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //

    AddFmuVariable((int*)(&config.use_SMC), "use_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVecVariable(config.init_loc, "init_loc", "m", "initial location",                                //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&config.init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial yaw angle",     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVecVariable(config.g_acc, "g_acc", "m/s2", "gravitational acceleration",                         //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&config.step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set I/O PARAMETERS for this FMU
    AddFmuVariable(&config.output_dir, "output_dir", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&config.fps, "fps", FmuVariable::Type::Real, "1", "rendering frequency",             //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINUOUS INPUTS for this FMU (driver inputs)
    AddFmuVariable(&driver_inputs.m_steering, "steering", FmuVariable::Type::Real, "1", "steering input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_throttle, "throttle", FmuVariable::Type::Real, "1", "throttle input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_braking, "braking", FmuVariable::Type::Real, "1", "braking input",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_clutch, "clutch", FmuVariable::Type::Real, "1", "clutch input",        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    // Set CONTINUOUS INPUTS for this FMU (terrain information for each wheel)
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;
        
        AddFmuVariable(&wheel_data[iw].terrain_height, prefix + ".terrain_height", FmuVariable::Type::Real, "m", 
                      prefix + " terrain height",
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);
        
        AddFmuVecVariable(wheel_data[iw].terrain_normal, prefix + ".terrain_normal", "1", 
                         prefix + " terrain normal",
                         FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);
        
        AddFmuVariable(&wheel_data[iw].terrain_mu, prefix + ".terrain_mu", FmuVariable::Type::Real, "1", 
                      prefix + " terrain friction coefficient",
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);
    }

    // Set DISCRETE INPUTS for this FMU (I/O)
    AddFmuVariable((int*)(&config.save_images), "save_images", FmuVariable::Type::Boolean, "1", "trigger saving images",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::discrete);               //

    // Set CONTINUOUS OUTPUTS for this FMU (vehicle reference frame)
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                          //
                              FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINOUS OUTPUTS for this FMU (wheel state)
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state.lin_vel = VNULL;
        wheel_data[iw].state.ang_vel = VNULL;

        std::string prefix = "wheel_" + wheel_data[iw].identifier;

        AddFmuVecVariable(wheel_data[iw].state.pos, prefix + ".pos", "m", prefix + " position",                      //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
        AddFmuQuatVariable(wheel_data[iw].state.rot, prefix + ".rot", "1", prefix + " rotation",                     //
                           FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);            //
        AddFmuVecVariable(wheel_data[iw].state.lin_vel, prefix + ".lin_vel", "m/s", prefix + " linear velocity",     //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //
        AddFmuVecVariable(wheel_data[iw].state.ang_vel, prefix + ".ang_vel", "rad/s", prefix + " angular velocity",  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //
    }

    // Add outputs for chassis position and orientation
    for (int i = Output::CHASSIS_POS_X; i <= Output::CHASSIS_ORIENT_Z; i++) {
        std::string name;
        std::string unit = "m";
        std::string desc;
        
        switch(i) {
            case Output::CHASSIS_POS_X:
                name = "chassis_pos_x";
                desc = "Chassis position X";
                break;
            case Output::CHASSIS_POS_Y:
                name = "chassis_pos_y";
                desc = "Chassis position Y";
                break;
            case Output::CHASSIS_POS_Z:
                name = "chassis_pos_z";
                desc = "Chassis position Z";
                break;
            case Output::CHASSIS_ORIENT_X:
                name = "chassis_orient_x";
                unit = "rad";
                desc = "Chassis orientation X";
                break;
            case Output::CHASSIS_ORIENT_Y:
                name = "chassis_orient_y";
                unit = "rad";
                desc = "Chassis orientation Y";
                break;
            case Output::CHASSIS_ORIENT_Z:
                name = "chassis_orient_z";
                unit = "rad";
                desc = "Chassis orientation Z";
                break;
        }
        
        AddFmuVariable(&output_values[i], name, FmuVariable::Type::Real, unit, desc,
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);
    }

    // Specify variable dependencies
    DeclareVariableDependencies("ref_frame", {"init_loc", "init_yaw"});
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(prefix + ".pos", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".rot", {"init_loc", "init_yaw"});
    }

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateVehicleOutputs(); });
}

void FmuVehicleComponent::CreateVehicle() {
    std::cout << "Creating vehicle FMU" << std::endl;
    std::cout << " Data path:         " << data_path << std::endl;
    std::cout << " Vehicle JSON:      " << config.vehicle_JSON << std::endl;
    std::cout << " Tire Front JSON:   " << config.tire_JSON_front << std::endl;
    std::cout << " Tire Rear JSON:    " << config.tire_JSON_rear << std::endl;
    std::cout << " Engine JSON:       " << config.engine_JSON << std::endl;
    std::cout << " Transmission JSON: " << config.transmission_JSON << std::endl;
    std::cout << " Initial location:  " << config.init_loc << std::endl;
    std::cout << " Initial yaw:       " << config.init_yaw << std::endl;

    vehicle::SetDataPath(data_path);

    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(config.vehicle_JSON,
                                                      config.use_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    vehicle->Initialize(ChCoordsys<>(config.init_loc, QuatFromAngleZ(config.init_yaw)));

    // Initialize the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    // Cache vehicle wheels
    wheel_data[0].wheel = vehicle->GetWheel(0, VehicleSide::LEFT);
    wheel_data[1].wheel = vehicle->GetWheel(0, VehicleSide::RIGHT);
    wheel_data[2].wheel = vehicle->GetWheel(1, VehicleSide::LEFT);
    wheel_data[3].wheel = vehicle->GetWheel(1, VehicleSide::RIGHT);

    // Create and attach tires
    std::shared_ptr<ChTire> tire_front_left = ReadTireJSON(config.tire_JSON_front);
    std::shared_ptr<ChTire> tire_front_right = ReadTireJSON(config.tire_JSON_front);
    std::shared_ptr<ChTire> tire_rear_left = ReadTireJSON(config.tire_JSON_rear);
    std::shared_ptr<ChTire> tire_rear_right = ReadTireJSON(config.tire_JSON_rear);
    
    vehicle->InitializeTire(tire_front_left, vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::MESH);
    vehicle->InitializeTire(tire_front_right, vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::MESH);
    vehicle->InitializeTire(tire_rear_left, vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::MESH);
    vehicle->InitializeTire(tire_rear_right, vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::MESH);

    // Set visualization types
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);
    vehicle->SetTireVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(config.engine_JSON);
    auto transmission = ReadTransmissionJSON(config.transmission_JSON);
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);
}

void FmuVehicleComponent::ConfigureSystem() {
    // Containing system
    auto system = vehicle->GetSystem();

    system->SetGravitationalAcceleration(config.g_acc);

    // Associate a collision system
    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Modify solver settings if the vehicle model contains bushings
    if (vehicle->HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);
        solver->SetVerbose(false);

        config.step_size = std::min(config.step_size, 2e-4);
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
}

void FmuVehicleComponent::SynchronizeVehicle(double time) {
    // First, synchronize the vehicle system
    vehicle->Synchronize(time, driver_inputs);
    
    // Apply terrain-based forces to each wheel
    for (int iw = 0; iw < 4; iw++) {
        // Get the tire and synchronize it with the terrain
        auto tire = wheel_data[iw].wheel->GetTire();
        if (tire) {
            tire->Synchronize(time, *terrain);
            
            // Get terrain force from tire and apply to wheel
            auto force = tire->ReportTireForce(terrain.get());
            wheel_data[iw].wheel->Synchronize(force);
        }
    }

#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        vis_sys->Synchronize(time, driver_inputs);
    }
#endif
}

void FmuVehicleComponent::CalculateVehicleOutputs() {
    // Extract wheel states
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state = wheel_data[iw].wheel->GetState();
    }

    // Update the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    // Update output array for chassis position and orientation
    output_values[Output::CHASSIS_POS_X] = vehicle->GetChassisBody()->GetPos().x();
    output_values[Output::CHASSIS_POS_Y] = vehicle->GetChassisBody()->GetPos().y();
    output_values[Output::CHASSIS_POS_Z] = vehicle->GetChassisBody()->GetPos().z();

    output_values[Output::CHASSIS_ORIENT_X] = vehicle->GetChassisBody()->GetRot().GetCardanAnglesXYZ().x();
    output_values[Output::CHASSIS_ORIENT_Y] = vehicle->GetChassisBody()->GetRot().GetCardanAnglesXYZ().y();
    output_values[Output::CHASSIS_ORIENT_Z] = vehicle->GetChassisBody()->GetRot().GetCardanAnglesXYZ().z();

    // Extract additional outputs for chassis velocities and accelerations
    const auto chassis_frame = vehicle->GetChassisBody()->GetCoordsys();
    const auto pos_dt = vehicle->GetChassisBody()->GetPosDt();
    output_values[Output::CHASSIS_VEL_X] = chassis_frame.TransformDirectionParentToLocal(pos_dt).x();
    output_values[Output::CHASSIS_VEL_Y] = chassis_frame.TransformDirectionParentToLocal(pos_dt).y();
    output_values[Output::CHASSIS_VEL_Z] = chassis_frame.TransformDirectionParentToLocal(pos_dt).z();

    output_values[Output::CHASSIS_ANG_VEL_X] = vehicle->GetChassisBody()->GetAngVelLocal().x();
    output_values[Output::CHASSIS_ANG_VEL_Y] = vehicle->GetChassisBody()->GetAngVelLocal().y();
    output_values[Output::CHASSIS_ANG_VEL_Z] = vehicle->GetChassisBody()->GetAngVelLocal().z();

    const auto pos_dtdt = vehicle->GetChassisBody()->GetLinAcc();
    output_values[Output::CHASSIS_ACC_X] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).x();
    output_values[Output::CHASSIS_ACC_Y] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).y();
    output_values[Output::CHASSIS_ACC_Z] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).z();

    output_values[Output::CHASSIS_ANG_ACC_X] = vehicle->GetChassisBody()->GetAngAccLocal().x();
    output_values[Output::CHASSIS_ANG_ACC_Y] = vehicle->GetChassisBody()->GetAngAccLocal().y();
    output_values[Output::CHASSIS_ANG_ACC_Z] = vehicle->GetChassisBody()->GetAngAccLocal().z();
}

void FmuVehicleComponent::_preModelDescriptionExport() {}

void FmuVehicleComponent::_postModelDescriptionExport() {}

void FmuVehicleComponent::_enterInitializationMode() {}

void FmuVehicleComponent::_exitInitializationMode() {
    // Create the vehicle system
    CreateVehicle();

    // Configure Chrono system
    ConfigureSystem();

    // Initialize the terrain interface
    terrain = chrono_types::make_shared<LocalTerrain>();
    terrain->wheel_data = &wheel_data;

    // Initialize runtime visualization (if requested and if available)
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        std::cout << " Enable run-time visualization" << std::endl;

        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetJPEGQuality(100);
        vis_sys->SetWindowTitle("Wheeled Vehicle FMU");
        vis_sys->SetWindowSize(800, 800);
        vis_sys->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), config.camera_distance, 0.5);
        vis_sys->AddGrid(0.5, 0.5, 2000, 400, ChCoordsys<>(config.init_loc, QuatFromAngleZ(config.init_yaw)),
                         ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
    }
#endif
}

fmi2Status FmuVehicleComponent::_doStep(fmi2Real currentCommunicationPoint,
                                      fmi2Real communicationStepSize,
                                      fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                            std::min(communicationStepSize, config.step_size));
        vehicle->Advance(h);

#ifdef CHRONO_IRRLICHT
        if (vis_sys) {
            auto status = vis_sys->Run();
            if (!status)
                return fmi2Discard;
            vis_sys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vis_sys->Render();
            vis_sys->RenderFrame(ref_frame);
            vis_sys->EndScene();

            if (config.save_images && m_time >= render_frame / config.fps) {
                std::ostringstream filename;
                filename << config.output_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".bmp";
                vis_sys->WriteImageToFile(filename.str());
                render_frame++;
            }

            vis_sys->Advance(h);
        }
#endif

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}

bool CreateVehicleFMU(const std::string& model_config_file, 
                      const std::string& output_directory,
                      const std::string& fmu_name) {
    try {
        // Load the model configuration using Chrono's JSON utilities
        FMU_VehicleModelConfig config(model_config_file);
        
        // Set up FMU parameters
        std::string fmu_model_identifier = fmu_name;
        std::string fmu_guid = "chrono-simulink-cosim-vehicle-fmu-guid-" + std::to_string(std::time(nullptr));
        
        // Set up FMU export parameters
        FmuExportParameters params;
        params.fmuName = fmu_name;
        params.modelIdentifier = fmu_model_identifier;
        params.guid = fmu_guid;
        params.fmuOutputDir = output_directory;
        params.fmuType = fmi2Type::fmi2CoSimulation;
        params.fmuVersion = "2.0";
        params.author = "Project Chrono";
        params.copyright = "Copyright 2023 ProjectChrono";
        params.license = "BSD 3-Clause";
        params.description = "Chrono Vehicle FMU";
        params.version = "1.0.0";
        
        // Add source files needed to build the FMU
        params.addSourceFile("src/vehicle_fmu/VehicleFMU.cpp");
        params.addSourceFile("src/vehicle_fmu/VehicleFMU.h");
        
        // Copy model configuration and related files to resources
        params.addResourceFile(model_config_file);
        if (!config.vehicle_JSON.empty()) {
            params.addResourceFile(config.vehicle_JSON);
        }
        if (!config.tire_JSON_front.empty()) {
            params.addResourceFile(config.tire_JSON_front);
        }
        if (!config.tire_JSON_rear.empty()) {
            params.addResourceFile(config.tire_JSON_rear);
        }
        if (!config.engine_JSON.empty()) {
            params.addResourceFile(config.engine_JSON);
        }
        if (!config.transmission_JSON.empty()) {
            params.addResourceFile(config.transmission_JSON);
        }
        
        // Generate the FMU
        std::cout << "Creating FMU: " << fmu_name << std::endl;
        std::cout << "Output directory: " << output_directory << std::endl;
        std::cout << "Model config file: " << model_config_file << std::endl;
        
        // Helper function to create an instance of the FMU
        params.setFmuCreationFunction(&fmi2Instantiate_getPointer);
        
        // Export the FMU
        exportFmu(params);
        
        std::cout << "FMU export completed successfully." << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error creating FMU: " << e.what() << std::endl;
        return false;
    }
}

// Implementation of LocalTerrain methods
double FmuVehicleComponent::LocalTerrain::GetHeight(const ChVector3d& loc) const {
    // Find the closest wheel and use its terrain height
    double min_dist = std::numeric_limits<double>::max();
    double height = 0.0;
    
    for (int i = 0; i < 4; i++) {
        ChVector3d wheel_pos = (*wheel_data)[i].wheel->GetSpindle()->GetPos();
        double dist = (wheel_pos - loc).Length();
        if (dist < min_dist) {
            min_dist = dist;
            height = (*wheel_data)[i].terrain_height;
        }
    }
    
    return height;
}

ChVector3d FmuVehicleComponent::LocalTerrain::GetNormal(const ChVector3d& loc) const {
    // Find the closest wheel and use its terrain normal
    double min_dist = std::numeric_limits<double>::max();
    ChVector3d normal = ChWorldFrame::Vertical();
    
    for (int i = 0; i < 4; i++) {
        ChVector3d wheel_pos = (*wheel_data)[i].wheel->GetSpindle()->GetPos();
        double dist = (wheel_pos - loc).Length();
        if (dist < min_dist) {
            min_dist = dist;
            normal = (*wheel_data)[i].terrain_normal;
        }
    }
    
    return normal;
}

float FmuVehicleComponent::LocalTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    // Find the closest wheel and use its terrain friction
    double min_dist = std::numeric_limits<double>::max();
    float friction = 0.8f;
    
    for (int i = 0; i < 4; i++) {
        ChVector3d wheel_pos = (*wheel_data)[i].wheel->GetSpindle()->GetPos();
        double dist = (wheel_pos - loc).Length();
        if (dist < min_dist) {
            min_dist = dist;
            friction = static_cast<float>((*wheel_data)[i].terrain_mu);
        }
    }
    
    return friction;
}

} // namespace vehicle_fmu
} // namespace chrono 