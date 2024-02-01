// =============================================================================
// Authors: Trevor Vidano
// Date: 01/29/2024
// =============================================================================
//
// Cosimulation of a suspension test rig with Simulink.
//
// The model uses inputs as the  left and right pose displacements. There is also
// the ability to use the steering input if the suspension model has one.
//
// The steering actuator is position-driven. The input is the angle of the 
// pinion, which is then converted to linear displacement of the rack. The motor
// torque commands are sent to each wheel's axle. The brake commands are 
// a torque applied in the opposite direction of the spindle's angular velocity.
//
// This was created as a modified combination of the demo_COSIM_hydraulics,
// and demo_VEH_SuspensionTestRig. Please refer to their source code 
// files to understand the individual components that were needed to make this 
// work.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_cosimulation/ChCosimulation.h"

#include <filesystem>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::utils;
using namespace chrono::cosimul;

// =============================================================================
// Class that enables direct access to the rig inputs.
class CosimSuspensionTestRig : public ChDriverSTR {
  public:
    /// Set the value for the driver left post displacement input.
    void SetDisplacementLeft(int axle, double val, double min_val = -10, double max_val = 10) {
        m_displLeft[axle] = ChClamp(val, min_val, max_val);
    }

    /// Set the value for the driver right post displacement input.
    void SetDisplacementRight(int axle, double val, double min_val = -10, double max_val = 10) {
        m_displRight[axle] = ChClamp(val, min_val, max_val);
    }
    
    /// Set the value for the driver steering input.
    void SetSteering(double val, double min_val = -10, double max_val = 10) {
        m_steering = ChClamp(val, min_val, max_val);
    }
};

// =============================================================================
// Specification of a vehicle suspension test rig
// Available models:
//    Generic : demonstrates STR for an axle with antiroll bar
//               (requires smaller step size)

class STR_Setup {
  public:
    virtual std::string SuspensionRigJSON() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string DataDriverFile() const = 0;
    virtual std::vector<int> TestAxles() const = 0;
    virtual std::vector<int> TestSubchassis() const = 0;
    virtual std::vector<int> TestSteerings() const = 0;
    virtual double InitRideHeight() const = 0;
    virtual double PostLimit() const = 0;
    virtual double CameraDistance() const = 0;
};

class Generic_STR_Setup : public STR_Setup {
  public:
    virtual std::string SuspensionRigJSON() const override { return "generic/suspensionTest/STR_example.json"; }
    virtual std::string VehicleJSON() const override { 
        return "ford_taurus_1994/Vehicle_ford_taurus_1994.json";
        // return "ford_expedition_2003/vehicle/Vehicle_ford_expedition_2003.json"; 
        // return "generic/vehicle/Vehicle_DoubleWishbones_ARB.json";
    }
    virtual std::string TireJSON() const override { return "ford_expedition_2003/tire/TMeasyTire.json"; }
    virtual std::string DataDriverFile() const override { return "generic/suspensionTest/ST_inputs.dat"; }
    virtual std::vector<int> TestAxles() const override { return {1}; }
    virtual std::vector<int> TestSubchassis() const override { return {}; }
    virtual std::vector<int> TestSteerings() const override { return {0}; }
    virtual double InitRideHeight() const override { return -0.001; } // 0.55
    virtual double PostLimit() const override { return 1.0; }
    virtual double CameraDistance() const override { return 2.0; }
};

// =============================================================================
// USER SETTINGS

Generic_STR_Setup setup;

// STR rig type
enum class RigMode {PLATFORM, PUSHROD};
RigMode rig_mode = RigMode::PUSHROD;

// Specification of test rig inputs
// enum class DriverMode {DATA_FILE, INTERACTIVE};
// DriverMode driver_mode = DriverMode::INTERACTIVE;

// Output collection
bool output = true;
bool plot = true;
std::string out_dir = GetChronoOutputPath() + "SUSPENSION_TEST_RIG";
double out_step_size = 1e-2;

// Simulation step size
double step_size = 1e-3;

// =============================================================================

// Function used to create a suspension test rig from a specific vehicle. The
// configuration is done by setting members of the Generic_STR_Setup class.
std::shared_ptr<ChSuspensionTestRig> CreateFromVehicleModel() {
    std::cout << "Using vehicle specification file: " << setup.VehicleJSON() << std::endl;

    // Create the vehicle
    auto vehicle =
        chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(setup.VehicleJSON()), ChContactMethod::SMC);

    // Create the suspension test rig
    std::shared_ptr<ChSuspensionTestRig> rig;
    switch (rig_mode) {
        default:
        case RigMode::PLATFORM: {
            rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, setup.TestAxles(), setup.PostLimit());
            break;
        }
        case RigMode::PUSHROD: {
            rig = chrono_types::make_shared<ChSuspensionTestRigPushrod>(vehicle, setup.TestAxles(), setup.PostLimit());
            break;
        }
    }

    // Include additional subsystems in test
    for (auto is : setup.TestSteerings())
        rig->IncludeSteeringMechanism(is);
    for (auto is : setup.TestSubchassis())
        rig->IncludeSubchassis(is);

    rig->SetInitialRideHeight(setup.InitRideHeight());

    return rig;
}

// Function used to create a suspension test rig from a suspension test rig
// configuration file (a json file).
std::shared_ptr<ChSuspensionTestRig> CreateFromSpecFile() {
    std::cout << "Using STR specification file: " << setup.SuspensionRigJSON() << std::endl;

    switch (rig_mode) {
        default:
        case RigMode::PLATFORM:
            return chrono_types::make_shared<ChSuspensionTestRigPlatform>(
                vehicle::GetDataFile(setup.SuspensionRigJSON()));
        case RigMode::PUSHROD:
            return chrono_types::make_shared<ChSuspensionTestRigPushrod>(
                vehicle::GetDataFile(setup.SuspensionRigJSON()));
    }
}

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

    // Option 1: Create the suspension rig from an existing vehicle model
    auto rig = CreateFromVehicleModel(); 

    // Option 2: Create the suspension rig from a JSON rig specification file
    //auto rig = CreateFromSpecFile();

    // Create and attach the vehicle tires.
    // (not needed if tires are specified in the vehicle's suspension JSON files)
    for (auto ia : setup.TestAxles()) {
        auto axle = rig->GetVehicle().GetAxle(ia);
        for (auto& wheel : axle->GetWheels()) {
            if (!wheel->GetTire()) {
                auto tire = ReadTireJSON(vehicle::GetDataFile(setup.TireJSON()));
                rig->GetVehicle().InitializeTire(tire, wheel, VisualizationType::NONE);
            }
        }
    }

    // Optional rig settings
    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    rig->SetTireVisualizationType(VisualizationType::MESH);

    // Create the vehicle Irrlicht application.
    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Suspension Test Rig");
    vis->SetChaseCamera(0.5 * (rig->GetSpindlePos(0, LEFT) + rig->GetSpindlePos(0, RIGHT)), setup.CameraDistance(), 0.5);

    // Create and attach the driver system.
    // switch (driver_mode) {
    //     case DriverMode::DATA_FILE: {
    //         auto driver = chrono_types::make_shared<ChDataDriverSTR>(vehicle::GetDataFile(setup.DataDriverFile()));
    //         rig->SetDriver(driver);
    //         break;
    //     }
    //     case DriverMode::INTERACTIVE: {
    //         auto driver = chrono_types::make_shared<ChIrrGuiDriverSTR>(*vis);
    //         driver->SetSteeringDelta(1.0 / 50);
    //         driver->SetDisplacementDelta(1.0 / 250);
    //         rig->SetDriver(driver);
    //         break;
    //     }
    // }
    auto driver = chrono_types::make_shared<CosimSuspensionTestRig>();
    rig->SetDriver(driver);

    // Initialize suspension test rig.
    rig->Initialize();

    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&rig->GetVehicle());

    // Create a cosimulation interface and exchange data with Simulink.
    try {
        // Prepare the two column vectors of data that will be swapped back 
        // and forth between Chrono and Simulink.
        int num_in = 3;
        int num_out = 16;
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

        // Simulation loop
        while (vis->Run()) {
            std::vector<int> i_axle = setup.TestAxles();
            // Advance simulation of the rig
            while (my_time < sim_time) {
                // Overwrite driver commands and use cosimulation inputs.
                // rig->UpdateActuators({data_in[0]}, {0.0}, {data_in[1]}, {0.0});
                driver->SetSteering(data_in[0]);
                driver->SetDisplacementLeft(0, data_in[1]);
                driver->SetDisplacementRight(0, data_in[2]);

                rig->Advance(step_size);

                // Update chrono clock.
                my_time = rig->GetVehicle().GetChTime();
            }

            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            // Update visualization app
            vis->Synchronize(rig->GetDriverMessage(), {rig->GetSteeringInput(), 0, 0});
            vis->Advance(step_size);

            // Send data over cosimulation connection.
            // Spring forces.
            data_out(0) = rig->GetVehicle().GetSuspension(i_axle[0])->ReportSuspensionForce(LEFT).spring_force;
            data_out(1) = rig->GetVehicle().GetSuspension(i_axle[0])->ReportSuspensionForce(RIGHT).spring_force;

            // Damper forces.
            data_out(2) = rig->GetVehicle().GetSuspension(i_axle[0])->ReportSuspensionForce(LEFT).shock_force;
            data_out(3) = rig->GetVehicle().GetSuspension(i_axle[0])->ReportSuspensionForce(RIGHT).shock_force;

            // Camber angles.
            data_out(4) = rig->GetVehicle().GetTire(i_axle[0], LEFT)->GetCamberAngle();
            data_out(5) = rig->GetVehicle().GetTire(i_axle[0], RIGHT)->GetCamberAngle();

            // Steer angles.
            ChVector<> wheel_normal = rig->GetVehicle().GetWheel(i_axle[0],LEFT)->GetState().rot.GetYaxis();
            ChVector<> normal = rig->GetVehicle().GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(6) = std::atan2(normal.x(), normal.y());
            wheel_normal = rig->GetVehicle().GetWheel(i_axle[0],RIGHT)->GetState().rot.GetYaxis();
            normal = rig->GetVehicle().GetChassis()->GetTransform().TransformDirectionParentToLocal(wheel_normal);
            data_out(7) = std::atan2(normal.x(), normal.y());

            // Spindle vertical displacement.
            double spindle_z = rig->GetVehicle().GetSpindlePos(i_axle[0], LEFT).z();
            double chassis_z = rig->GetVehicle().GetChassis()->GetPos().z();
            data_out(8) = chassis_z - spindle_z;
            spindle_z = rig->GetVehicle().GetSpindlePos(i_axle[0], RIGHT).z();
            data_out(9) = chassis_z - spindle_z;

            cosim_interface.SendData(my_time, data_out);  // --> to Simulink

            // Receive inputs.
            cosim_interface.ReceiveData(sim_time, data_in);  // <-- from Simulink
        }
    } catch (ChExceptionSocket exception) {
        GetLog() << " ERROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}