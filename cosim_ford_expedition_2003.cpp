// =============================================================================
// Authors: Trevor Vidano
// Date: 01/26/2024
// =============================================================================
//
// Main driver function for the Ford Expedition 2003 co-simulated with Simulink.
// This model expects Simulink to provide the driver inputs to the vehicle: 
// steering pinion angle, axle torque, and brake torque for each wheel. This
// choice of inputs enables advanced control functionality to be implemented in
// Simulink such as traction control, anti-lock braking, differential braking, 
// etc. as well as powertrain models, driveline models, and brake models. There
// are a large number of outputs. They are sent to simulink as a vector for 
// simplicity.
//
// The steering actuator is position-driven. The input is the angle of the 
// pinion, which is then converted to linear displacement of the rack. The motor
// torque commands are sent to each wheel's axle. The brake commands are 
// a torque applied in the opposite direction of the spindle's angular velocity.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// This was created as a modified combination of the demo_COSIM_hydraulics,
// demo_VEH_WheeledJSON, and demo_VEH_Sedan. Please refer to their source code 
// files to understand the individual components that were needed to make this 
// work.
//
// =============================================================================

#include "chrono/core/ChStream.h"

#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_cosimulation/ChCosimulation.h"

#include <filesystem>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::utils;
using namespace chrono::cosimul;

// =============================================================================

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    // virtual std::string PowertrainJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual ChContactMethod ContactMethod() const = 0;
};

class Expedition_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Ford Expedition 2003"; }
    virtual std::string VehicleJSON() const override { return "ford_expedition_2003/Vehicle_ford_expedition_2003.json"; }
    virtual std::string TireJSON() const override {
        return "ford_expedition_2003/Expedition_TMeasyTire.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const { return ChContactMethod::SMC; }
};

class Taurus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Ford Taurus 1994"; }
    virtual std::string VehicleJSON() const override { return "ford_taurus_1994/Vehicle_ford_taurus_1994.json"; }
    virtual std::string TireJSON() const override {
        //return "ford_taurus_1994/Taurus_TMeasyTire.json";
        return "generic/tire/FialaTire.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const { return ChContactMethod::SMC; }
};

auto vehicle_model = Expedition_Model();

// JSON files for terrain.v
//std::string rigidterrain_file("terrain/RigidPlane.json");
std::string rigidterrain_file("terrain/RigidPlane1p0.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle displacement above the ground.
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot = Q_from_Euler123({0.0, 0.0, 0.0});

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
// Here the step_size must be the same as the sampling period that is
// entered in the Simulink Cosimulation block.
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 20;  // FPS = 20

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
    GetLog() << "Using Port Number: " << std::to_string(PORT_NUMBER) << "\n";

    // --------------
    // Create systems
    // --------------

    // Create the vehicle system
    WheeledVehicle car(GetDataFile(vehicle_model.VehicleJSON()), vehicle_model.ContactMethod());
    car.Initialize(ChCoordsys<>(initLoc, initRot));
    car.GetChassis()->SetFixed(false);
    car.SetChassisVisualizationType(chassis_vis_type);
    //car.SetChassisRearVisualizationType(chassis_vis_type);
    car.SetSuspensionVisualizationType(suspension_vis_type);
    car.SetSteeringVisualizationType(steering_vis_type);
    car.SetWheelVisualizationType(wheel_vis_type);
    car.LockAxleDifferential(0, false);

    // No powertrain model is required since the axle torques come from the 
    // Cosimulation interface.
    // auto powertrain = ReadPowertrainJSON(GetDataFile(vehicle_model.PowertrainJSON()));
    // car.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : car.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetDataFile(vehicle_model.TireJSON()));
            car.InitializeTire(tire, wheel, tire_vis_type);
        }
    }

    // Containing system
    auto system = car.GetSystem();

    // Modify solver settings if the vehicle model contains bushings
    if (car.HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
        solver->SetVerbose(false);

        step_size = 2e-4;
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
    // You may want to use the EULER_IMPLICIT_LINEARIZED anyway. Uncomment below.
    // car.GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Create the terrain (from JSON file)
    RigidTerrain terrain(system, GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("cosim_ford_expedition_2003");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&car);

    // ------------------------
    // Create the driver system
    // ------------------------

    ChDriver driver(car);

    // ------------------------
    // Initialize values for simulation loop.
    // ------------------------

    car.LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << car.GetMass() << std::endl;
    std::cout << "\nWheelbase: " << car.GetWheelbase() << std::endl;
    std::cout << "\nFront Track: " << car.GetWheeltrack(0) << std::endl;
    std::cout << "\nRear Track: " << car.GetWheeltrack(1) << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    // Enabling realtime attempts to make the simulation run at real time, even if
    // the simulation can run faster.
    car.EnableRealtime(false);

    // Create a cosimulation interface and exchange data with Simulink.
    try {
        // Prepare the two column vectors of data that will be swapped back 
        // and forth between Chrono and Simulink.
        int num_in = 9;
        int num_out = 55;
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

        // Simulate the first 5 seconds before so that the vehicle begins
        // at rest and at steady-state (or at least close to it).
        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();
        const double steady_state_time = 0.0;
        // while (my_time < steady_state_time) {
        //     // Update modules (process inputs from other modules)
        //     driver.Synchronize(my_time);
        //     terrain.Synchronize(my_time);
        //     car.Synchronize(my_time, driver_inputs, terrain);
            
        //     // Advance simulation for one timestep for all modules
        //     driver.Advance(step_size);
        //     terrain.Advance(step_size);
        //     car.Advance(step_size);
        //     vis->Advance(step_size);
            
        //     // Increment chrono clock.
        //     my_time += step_size;
        // }

        // // Reset chrono clock after reaching steady-state.
        // my_time = 0.0;

        // 4) Run the co-simulation
        while (vis->Run()) {
            // A) ----------------- ADVANCE THE Chrono SIMULATION

            // Get driver inputs
            driver_inputs = driver.GetInputs();

            // Initialize variables that will be filtered for output.
            double acc_x(0);
            double acc_y(0);
            double acc_z(0);

            // Take multiple simulation steps until Chrono time matches the 
            // sim time.
            while (my_time < sim_time) {
                // Update modules (process inputs from other modules)
                driver.Synchronize(my_time);
                terrain.Synchronize(my_time);
                car.Synchronize(my_time, driver_inputs, terrain);

                // Override driveline torque application.
                car.GetDriveline()->GetDriveshaft()->SetAppliedTorque(0.0);
                car.GetSuspension(0)->GetAxle(LEFT)->SetAppliedTorque(-data_in(1));
                car.GetSuspension(0)->GetAxle(RIGHT)->SetAppliedTorque(-data_in(2));
                car.GetSuspension(1)->GetAxle(LEFT)->SetAppliedTorque(-data_in(3));
                car.GetSuspension(1)->GetAxle(RIGHT)->SetAppliedTorque(-data_in(4));

                // Override individual brake actuation.
                car.GetBrake(0, LEFT)->Synchronize(data_in(5));
                car.GetBrake(0, RIGHT)->Synchronize(data_in(6));
                car.GetBrake(1, LEFT)->Synchronize(data_in(7));
                car.GetBrake(1, RIGHT)->Synchronize(data_in(8));

                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                terrain.Advance(step_size);
                car.Advance(step_size);
                vis->Advance(step_size);

                // Increment chrono clock.
                my_time = car.GetChTime() - steady_state_time;

                // Get signals that will be filtered.
                Coordsys chassis_frame = car.GetChassisBody()->coord;
                ChVector<double> pos_dtdt = car.GetChassisBody()->GetPos_dtdt();
                acc_x = ax_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).x());
                acc_y = ay_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).y());
                acc_z = az_filt.Filter(chassis_frame.TransformDirectionParentToLocal(pos_dtdt).z());
            }
            step_number++;

            // Render scene and output POV-Ray data
            if (step_number % render_steps == 0) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                render_frame++;
            }

            // B) ----------------- SYNCHRONIZATION

            // B.1) - SEND data

            // - Set the Chrono variables into the vector that must
            //   be sent to Simulink at the next timestep:
            
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
            data_out(18) = car.GetSpindleOmega(0, LEFT);
            data_out(19) = car.GetSpindleOmega(0, RIGHT);
            data_out(20) = car.GetSpindleOmega(1, LEFT);
            data_out(21) = car.GetSpindleOmega(1, RIGHT);

            // Tire longitudinal slip ratio. (checked)
            data_out(22) = car.GetAxle(0)->GetWheel(LEFT)->GetTire()->GetLongitudinalSlip();
            data_out(23) = car.GetAxle(0)->GetWheel(RIGHT)->GetTire()->GetLongitudinalSlip();
            data_out(24) = car.GetAxle(1)->GetWheel(LEFT)->GetTire()->GetLongitudinalSlip();
            data_out(25) = car.GetAxle(1)->GetWheel(RIGHT)->GetTire()->GetLongitudinalSlip();

            // Tire lateral slip angle. (checked)
            data_out(26) = car.GetAxle(0)->GetWheel(LEFT)->GetTire()->GetSlipAngle();
            data_out(27) = car.GetAxle(0)->GetWheel(RIGHT)->GetTire()->GetSlipAngle();
            data_out(28) = car.GetAxle(1)->GetWheel(LEFT)->GetTire()->GetSlipAngle();
            data_out(29) = car.GetAxle(1)->GetWheel(RIGHT)->GetTire()->GetSlipAngle();

            // Longitudinal tire forces in the tire frame.
            data_out(30) = car.GetAxle(0)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.x();
            data_out(31) = car.GetAxle(0)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.x();
            data_out(32) = car.GetAxle(1)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.x();
            data_out(33) = car.GetAxle(1)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.x();
            
            // Lateral tire forces in the tire frame.
            data_out(34) = car.GetAxle(0)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.y();
            data_out(35) = car.GetAxle(0)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.y();
            data_out(36) = car.GetAxle(1)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.y();
            data_out(37) = car.GetAxle(1)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.y();
            
            // Vertical tire forces in the tire frame.
            data_out(38) = car.GetAxle(0)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.z();
            data_out(39) = car.GetAxle(0)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.z();
            data_out(40) = car.GetAxle(1)->GetWheel(LEFT)->GetTire()->ReportTireForce(&terrain).force.z();
            data_out(41) = car.GetAxle(1)->GetWheel(RIGHT)->GetTire()->ReportTireForce(&terrain).force.z();

            // Wheel torque applied from driveline. (checked)
            data_out(42) = car.GetDriveline()->GetSpindleTorque(0, LEFT);
            data_out(43) = car.GetDriveline()->GetSpindleTorque(0, RIGHT);
            data_out(44) = car.GetDriveline()->GetSpindleTorque(1, LEFT);
            data_out(45) = car.GetDriveline()->GetSpindleTorque(1, RIGHT);

            // Wheel torque applied from brakes. (checked)
            data_out(46) = car.GetBrake(0, LEFT)->GetBrakeTorque();
            data_out(47) = car.GetBrake(0, RIGHT)->GetBrakeTorque();
            data_out(48) = car.GetBrake(1, LEFT)->GetBrakeTorque();
            data_out(49) = car.GetBrake(1, RIGHT)->GetBrakeTorque();

            // Steering pinion angle.
            data_out(50) = driver.GetSteering();

            // Road wheels steer angle (angle made between wheel normal axis and chassis y plane).
            ChVector<> wheel_normal = car.GetWheel(0,LEFT)->GetState().rot.GetYaxis();
            ChVector<> normal = car.GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(51) = std::atan2(normal.x(), normal.y());
            wheel_normal = car.GetWheel(0,RIGHT)->GetState().rot.GetYaxis();
            normal = car.GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(52) = std::atan2(normal.x(), normal.y());
            wheel_normal = car.GetWheel(1,LEFT)->GetState().rot.GetYaxis();
            normal = car.GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(53) = std::atan2(normal.x(),normal.y());
            wheel_normal = car.GetWheel(1,RIGHT)->GetState().rot.GetYaxis();
            normal = car.GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(54) = std::atan2(normal.x(),normal.y());

            cosim_interface.SendData(my_time, data_out);  // --> to Simulink

            // B.2) - RECEIVE data
            cosim_interface.ReceiveData(sim_time, data_in);  // <-- from Simulink

            // - Update the Chrono system with the data received from Simulink.
            driver.SetSteering(data_in(0));
            driver.SetBraking(0.); // use no braking at this point, it is overriden in the while loop.

        }
    } catch (ChExceptionSocket exception) {
        GetLog() << " ERROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
