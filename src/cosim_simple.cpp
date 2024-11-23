// =============================================================================
// Authors: Trevor Vidano
// Date: 01/17/2024
// =============================================================================
//
// Main driver function for the Sedan full model cosimulated with Simulink. This
// model expects Simulink to provide the driver inputs to the vehicle: steering, 
// throttle, and braking. There are a large number of outputs. They are sent to
// simulink as a vector for simplicity.
//
// The actuators are very simple. The steering actuator is position-driven. The
// throttle is torque driven, but the torque can be ramped instantaneously. The
// brake command is equally split to all wheels and can also be ramped instantly.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// This was created as a modified combination of the demo_COSIM_hydraulics and 
// demo_VEH_Sedan. Please refer to their source code files to understand the
// individual components that were needed to make this work.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_cosimulation/ChCosimulation.h"

#include <filesystem>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;
using namespace chrono::utils;
using namespace chrono::cosimul;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY, PAC02)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
// Here the step_size must be the same as the sampling period that is
// entered in the Simulink Cosimulation block.
double step_size = 1e-3;
double tire_step_size = 1e-4;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 20;  // FPS = 20

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// Lowpass filters for eliminating simulation artifacts in acceleration data.
// The tire models used in this simulation are only valid up to moderate 
// frequencies anyway.
double cutoff_freq(30); // Hz
ChButterworth_Lowpass ax_filt(10, step_size, cutoff_freq);
ChButterworth_Lowpass ay_filt(10, step_size, cutoff_freq);
ChButterworth_Lowpass az_filt(10, step_size, cutoff_freq);

// =============================================================================

int main(int argc, char* argv[]) {
    // The first item in argv is the path to current executable. Use this to set
    // the Chrono Data Directory.
    std::filesystem::path path(argv[0]);
    std::filesystem::path grandparent_path = path.parent_path().parent_path();
    std::filesystem::path data_dir_path(grandparent_path.string());
    data_dir_path.append("data").append("");
    std::filesystem::path veh_data_path(data_dir_path.string());
    veh_data_path.append("vehicle").append("");
    SetChronoDataPath(data_dir_path.string());
    SetDataPath(veh_data_path.string());

    // If available, use the second argv as the port.
    int PORT_NUMBER = 50009;
    if ( argc == 2 ) {
        PORT_NUMBER = std::stoi(argv[1]);
    }
    GetLog() << "Using Port Number: " << std::to_string(PORT_NUMBER) << ".\n";

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize.
    // This vehicle has a rack and pinion steering actuator. The inputs range from
    // -1 to 1, which correspond to +/- 25 degrees.
    Sedan car;
    car.SetContactMethod(contact_method);
    car.SetChassisCollisionType(chassis_collision_type);
    car.SetChassisFixed(false);
    car.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    car.SetTireType(tire_model);
    car.SetTireStepSize(tire_step_size);
    // Use Tesla Model 3 specs for drag (2024).
    car.SetAerodynamicDrag(0.23, 1.44*1.85, 1.3); // (Cd, height*width, kg*m^-3)
    car.Initialize();

    car.SetChassisVisualizationType(chassis_vis_type);
    car.SetSuspensionVisualizationType(suspension_vis_type);
    car.SetSteeringVisualizationType(steering_vis_type);
    car.SetWheelVisualizationType(wheel_vis_type);
    car.SetTireVisualizationType(tire_vis_type);
    car.LockAxleDifferential(0, true);

    car.GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Create the terrain
    RigidTerrain terrain(car.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
                                     128, 128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("cosim_simple");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&car.GetVehicle());

    // ------------------------
    // Create the driver system
    // ------------------------

    ChDriver driver(car.GetVehicle());

    // ------------------------
    // Initialize values for simulation loop.
    // ------------------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        car.LogHardpointLocations();
    }

    car.GetVehicle().LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << car.GetVehicle().GetMass() << std::endl;
    std::cout << "\nWheelbase: " << car.GetVehicle().GetWheelbase() << std::endl;
    std::cout << "\nFront Track: " << car.GetVehicle().GetWheeltrack(0) << std::endl;
    std::cout << "\nRear Track: " << car.GetVehicle().GetWheeltrack(1) << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    if (contact_vis) {
        vis->SetSymbolScale(1e-4);
        vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    }

    // Enabling realtime attempts to make the simulation run at real time, even if
    // the simulation can run faster.
    car.GetVehicle().EnableRealtime(false);

    try {
        // Prepare the two column vectors of data that will be swapped back 
        // and forth between Chrono and Simulink. In detail we will:
        // - receive 3 variables from Simulink (steering, throttle, brake)
        // - send  variables to Simulink.
        int num_in = 3;
        int num_out = 39;
        ChVectorDynamic<double> data_in(num_in);
        ChVectorDynamic<double> data_out(num_out);
        data_in.setZero();
        data_out.setZero();

        // 1) Add a socket framework object.
        ChSocketFramework socket_tools;

        // 2) Create the cosimulation interface.
        ChCosimulation cosim_interface(socket_tools, num_in, num_out);

        // 3) Wait client (Simulink) to connect...
        GetLog() << " *** Waiting Simulink to start... *** \n     (load 'Matlab/simple_cosimulation.slx' in "
                    "Simulink and press Start...)\n\n";

        cosim_interface.WaitConnection(PORT_NUMBER);

        // Create my time and simulink time to synchronize.
        double my_time = 0;
        double sim_time = 0;

        // 4) Run the co-simulation
        while (vis->Run()) {
            // A) ----------------- ADVANCE THE Chrono SIMULATION

            // double time = car.GetSystem()->GetChTime();

            // End simulation
            if (my_time >= t_end)
                break;

            // Render scene and output POV-Ray data
            if (step_number % render_steps == 0) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                render_frame++;
            }

            // Debug logging
            if (debug_output && step_number % debug_steps == 0) {
                GetLog() << "\n\n============ System Information ============\n";
                GetLog() << "Time = " << my_time << "\n\n";
                car.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
            }

            // Get driver inputs
            DriverInputs driver_inputs = driver.GetInputs();

            // Initialize variables that will be filtered.
            double acc_x(0);
            double acc_y(0);
            double acc_z(0);

            while (my_time < sim_time) {
                // Update modules (process inputs from other modules)
                driver.Synchronize(my_time);
                terrain.Synchronize(my_time);
                car.Synchronize(my_time, driver_inputs, terrain);
                // vis->Synchronize(driver.GetInputModeAsString(), driver_inputs);

                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                terrain.Advance(step_size);
                car.Advance(step_size);
                vis->Advance(step_size);

                // Increment chrono clock.
                my_time += step_size;

                // Get signals that will be filtered.
                Coordsys chassis_frame = car.GetChassisBody()->coord;
                ChVector<double> pos_dtdt = car.GetChassisBody()->GetPos_dtdt();
                acc_x = ax_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).x());
                acc_y = ay_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).y());
                acc_z = az_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).z());
            }
            step_number++;

            // B) ----------------- SYNCHRONIZATION

            // B.1) - SEND data

            // - Set the Chrono variables into the vector that must
            //   be sent to Simulink at the next timestep:
            ChVector<double> cg_location(0,0,0);
            
            // Chassis position w.r.t. global frame. (checked)
            data_out(0) = car.GetChassis()->GetPos().x();
            data_out(1) = car.GetChassis()->GetPos().y();
            data_out(2) = car.GetChassis()->GetPos().z();
            
            // Chassis orientation (Euler Angles). (checked)
            data_out(3) = car.GetChassisBody()->GetRot().Q_to_Euler123().x();
            data_out(4) = car.GetChassisBody()->GetRot().Q_to_Euler123().y();
            data_out(5) = car.GetChassisBody()->GetRot().Q_to_Euler123().z();
            
            // Chassis velocity w.r.t. body-fixed frame. (checked)
            Coordsys chassis_frame = car.GetChassisBody()->coord;
            ChVector<double> pos_dt = car.GetChassisBody()->GetPos_dt();
            data_out(6) = chassis_frame.TransformDirectionParentToLocal(pos_dt).x();
            data_out(7) = chassis_frame.TransformDirectionParentToLocal(pos_dt).y();
            data_out(8) = chassis_frame.TransformDirectionParentToLocal(pos_dt).z();
            
            // Chassis angular velocity. (checked)
            data_out(9) = car.GetChassisBody()->GetWvel_loc().x();
            data_out(10) = car.GetChassisBody()->GetWvel_loc().y();
            data_out(11) = car.GetChassisBody()->GetWvel_loc().z();

            // Chassis acceleration w.r.t. body-fixed frame. (checked)
            data_out(12) = acc_x;
            data_out(13) = acc_y;
            data_out(14) = acc_z;

            // Chassis angular acceleration. (checked)
            data_out(15) = car.GetChassisBody()->GetWacc_loc().x();
            data_out(16) = car.GetChassisBody()->GetWacc_loc().y();
            data_out(17) = car.GetChassisBody()->GetWacc_loc().z();

            // Wheel angular velocity. (could also check GetSpindleAngVel)
            data_out(18) = car.GetVehicle().GetSpindleOmega(0, LEFT);
            data_out(19) = car.GetVehicle().GetSpindleOmega(0, RIGHT);
            data_out(20) = car.GetVehicle().GetSpindleOmega(1, LEFT);
            data_out(21) = car.GetVehicle().GetSpindleOmega(1, RIGHT);

            // Tire longitudinal slip ratio. (checked)
            data_out(22) = car.GetVehicle().GetAxle(0)->GetWheel(LEFT)->GetTire()->GetLongitudinalSlip();
            data_out(23) = car.GetVehicle().GetAxle(0)->GetWheel(RIGHT)->GetTire()->GetLongitudinalSlip();
            data_out(24) = car.GetVehicle().GetAxle(1)->GetWheel(LEFT)->GetTire()->GetLongitudinalSlip();
            data_out(25) = car.GetVehicle().GetAxle(1)->GetWheel(RIGHT)->GetTire()->GetLongitudinalSlip();

            // Tire lateral slip angle. (checked)
            data_out(26) = car.GetVehicle().GetAxle(0)->GetWheel(LEFT)->GetTire()->GetSlipAngle();
            data_out(27) = car.GetVehicle().GetAxle(0)->GetWheel(RIGHT)->GetTire()->GetSlipAngle();
            data_out(28) = car.GetVehicle().GetAxle(1)->GetWheel(LEFT)->GetTire()->GetSlipAngle();
            data_out(29) = car.GetVehicle().GetAxle(1)->GetWheel(RIGHT)->GetTire()->GetSlipAngle();

            // Wheel torque applied from driveline. (checked)
            data_out(30) = car.GetVehicle().GetDriveline()->GetSpindleTorque(0, LEFT);
            data_out(31) = car.GetVehicle().GetDriveline()->GetSpindleTorque(0, RIGHT);
            data_out(32) = car.GetVehicle().GetDriveline()->GetSpindleTorque(1, LEFT);
            data_out(33) = car.GetVehicle().GetDriveline()->GetSpindleTorque(1, RIGHT);

            // Wheel torque applied from brakes. (checked)
            data_out(34) = car.GetVehicle().GetBrake(0, LEFT)->GetBrakeTorque();
            data_out(35) = car.GetVehicle().GetBrake(0, RIGHT)->GetBrakeTorque();
            data_out(36) = car.GetVehicle().GetBrake(1, LEFT)->GetBrakeTorque();
            data_out(37) = car.GetVehicle().GetBrake(1, RIGHT)->GetBrakeTorque();

            // Steering pinion angle.
            double max_angle = car.GetVehicle().GetMaxSteeringAngle();
            data_out(38) = driver.GetSteering() / max_angle;

            cosim_interface.SendData(my_time, data_out);  // --> to Simulink

            // B.2) - RECEIVE data
            cosim_interface.ReceiveData(sim_time, data_in);  // <-- from Simulink

            // - Update the Chrono system with the data received from Simulink.
            driver.SetSteering(data_in(0));
            driver.SetThrottle(data_in(1));
            driver.SetBraking(data_in(2));

        }
    } catch (ChExceptionSocket exception) {
        GetLog() << " ERROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
