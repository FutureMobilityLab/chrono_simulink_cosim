// =============================================================================
// Authors: Trevor Vidano
//
// Date: 11/26/2024
// =============================================================================
//
// This file creates a Ford Expedition 2003 with a rack and pinion subsystem
// that uses a linear force as an input. The throttle and brakes are left as
// zero so that the vehicle simply is created and a sinusoidal input to the rack
// and pinion moves the steering.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"

#include "src/simulation_interface/SimulationInterface.h"


int main(int argc, char *argv[])
{
  auto simulation_interface = simulation_interface::SimulationInterface("sedan");

  auto vis = chrono_types::make_shared<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("Rack and pinion demo");
  const chrono::ChVector3<> trackPoint(0.0, 0.0, 1.75);
  vis->SetChaseCamera(trackPoint, 5.0, 0.5);
  vis->Initialize();
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddLogo();

  simulation_interface.SetVis(vis);
  
  auto driver = chrono_types::make_shared<chrono::vehicle::ChInteractiveDriverIRR>(*vis);
  driver->SetSteeringDelta(0.02);
  // driver->SetGains(10.0);
  driver->SetThrottleDelta(0.02);
  driver->SetBrakingDelta(0.06);
  driver->Initialize();
  simulation_interface.SetDriver(driver);

  double input[simulation_interface::Input::LENGTH] = {0.0, 0.0, 0.0};
  double output[simulation_interface::Output::LENGTH];
  while (vis->Run()) {
    simulation_interface.Step(input, output);
  }
  return 0;
}
