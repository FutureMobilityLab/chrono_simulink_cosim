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
#include "chrono_vehicle/wheeled_vehicle/suspension/ChThreeLinkIRS.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_cosimulation/ChCosimulation.h"

#include <filesystem>

#include "src/suspension_test_rig/SuspensionTestRig.h"

class Generic_STR_Setup : public chrono::vehicle::STR_Setup
{
public:
  virtual std::string SuspensionRigJSON() const override { return "generic/suspensionTest/STR_example.json"; }
  virtual std::string VehicleJSON() const override
  {
    // return "ford_taurus_1994/Vehicle_ford_taurus_1994.json";
    return "ford_expedition_2003/Vehicle_ford_expedition_2003.json";
    // return "generic/vehicle/Vehicle_DoubleWishbones_ARB.json";
  }
  virtual std::string TireJSON() const override { return "ford_expedition_2003/Expedition_TMeasyTire.json"; }
  virtual std::string DataDriverFile() const override { return "generic/suspensionTest/ST_inputs.dat"; }
  virtual std::vector<int> TestAxles() const override { return {0}; }
  virtual std::vector<int> TestSubchassis() const override { return {}; }
  virtual std::vector<int> TestSteerings() const override { return {0}; }
  virtual double InitRideHeight() const override { return -0.001; } // 0.55
  virtual double PostLimit() const override { return 1.0; }
  virtual double CameraDistance() const override { return 2.0; }
};

// =============================================================================
// USER SETTINGS

// auto setup = std::make_shared<Generic_STR_Setup>();
Generic_STR_Setup setup;

auto rig_mode = chrono::vehicle::RigMode::PLATFORM;

// Specification of test rig inputs
// enum class DriverMode {DATA_FILE, INTERACTIVE};
// DriverMode driver_mode = DriverMode::INTERACTIVE;

// Output collection
bool output = true;
bool plot = true;
std::string out_dir = chrono::GetChronoOutputPath() + "SUSPENSION_TEST_RIG";
double out_step_size = 1e-2;

// Simulation step size
double step_size = 1e-3;

// =============================================================================

int main(int argc, char *argv[])
{
  // The first item in argv is the path to current executable. Use this to set
  // the Chrono Data Directory.
  std::filesystem::path path(argv[0]);
  std::filesystem::path grandparent_path = path.parent_path().parent_path();
  std::filesystem::path data_dir_path(grandparent_path.string());
  data_dir_path.append("data").append("");
  std::filesystem::path veh_data_path(data_dir_path.string());
  veh_data_path.append("vehicle").append("");
  chrono::SetChronoDataPath(data_dir_path.string());
  chrono::vehicle::SetDataPath(veh_data_path.string());

  // Option 1: Create the suspension rig from an existing vehicle model
  auto rig = chrono::vehicle::CreateFromVehicleModel(rig_mode, &setup);

  // Option 2: Create the suspension rig from a JSON rig specification file
  // auto rig = CreateFromSpecFile();

  // Create and attach the vehicle tires.
  // (not needed if tires are specified in the vehicle's suspension JSON files)
  for (auto ia : setup.TestAxles())
  {
    auto axle = rig->GetVehicle().GetAxle(ia);
    for (auto &wheel : axle->GetWheels())
    {
      if (!wheel->GetTire())
      {
        auto tire = chrono::vehicle::ReadTireJSON(chrono::vehicle::GetDataFile(setup.TireJSON()));
        rig->GetVehicle().InitializeTire(tire, wheel, chrono::vehicle::VisualizationType::NONE);
      }
    }
  }

  // Optional rig settings
  rig->SetSuspensionVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  rig->SetSteeringVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  rig->SetSubchassisVisualizationType(chrono::vehicle::VisualizationType::PRIMITIVES);
  rig->SetWheelVisualizationType(chrono::vehicle::VisualizationType::NONE);
  rig->SetTireVisualizationType(chrono::vehicle::VisualizationType::MESH);

  // Create the vehicle Irrlicht application.
  auto vis = chrono_types::make_shared<chrono::vehicle::ChVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("Suspension Test Rig");
  vis->SetChaseCamera(0.5 * (rig->GetSpindlePos(0, chrono::vehicle::LEFT) + rig->GetSpindlePos(0, chrono::vehicle::RIGHT)), setup.CameraDistance(), 0.5);

  auto driver = chrono_types::make_shared<chrono::vehicle::CosimSuspensionTestRig>();
  rig->SetDriver(driver);

  // Initialize suspension test rig.
  rig->Initialize();

  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();
  vis->AttachVehicle(&rig->GetVehicle());

  // Simulation loop
  while (vis->Run())
  {
    std::vector<int> i_axle = setup.TestAxles();

    // Overwrite driver commands and use cosimulation inputs.
    // rig->UpdateActuators({data_in[0]}, {0.0}, {data_in[1]}, {0.0});
    driver->SetSteering(0.0);
    driver->SetDisplacementLeft(0, 0.0);
    driver->SetDisplacementRight(0, 0.0);

    rig->Advance(step_size);

    // Render scene
    vis->BeginScene();
    vis->Render();
    vis->EndScene();

    // Update visualization app
    vis->Synchronize(rig->GetDriverMessage(), {rig->GetSteeringInput(), 0, 0});
    vis->Advance(step_size);
  }

  return 0;
}