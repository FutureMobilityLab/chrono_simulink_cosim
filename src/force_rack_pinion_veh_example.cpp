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
#include "chrono/utils/ChConstants.h"
#include "chrono_vehicle/ChWorldFrame.h"

#include <iostream>
#include <cmath>

// Function to calculate terrain height as a function of radius from center
typedef double (*ground_height_func_t)(const chrono::ChVector3d&);

double CalculateRadialWaveGroundHeight(const double x, const double y) {
  // Center of the terrain pattern
  const double center_x = 0.0;
  const double center_y = 0.0;
  
  // Calculate radius from center
  double radius = std::sqrt(std::pow(x - center_x, 2) + std::pow(y - center_y, 2));
  
  // Parameters for radial sine wave
  const double amplitude = 1.0;
  const double wavelength = 20.0;
  
  // Calculate height as sine function of radius
  double height = amplitude * std::sin(2.0 * chrono::CH_PI * radius / wavelength) - 1.0;
  // std::cout << "height: " << height << "\n";
  return height;
}

double CalculateFlatGroundHeight(const chrono::ChVector3d& loc) {
  return 0.0;
}

double CalculateRampUpGroundHeight(const chrono::ChVector3d& loc) {
  return 0.5 * loc.x() - 1.0;
}

struct Terrain {
  double height;
  double nx;
  double ny;
  double nz;
};

// Function to calculate the terrain normal vector at a given point
constexpr double delta = 0.01;
constexpr double range = 0.2;
void FillTerrain(
  const double x, const double y, 
  const ground_height_func_t ground_height_func, 
  Terrain& terrain) {
  chrono::ChVector3d loc = chrono::ChVector3d(x, y, 0.0);
  chrono::ChVector3d loc_ISO = chrono::vehicle::ChWorldFrame::ToISO(loc);
  // to avoid 'jumping' of the normal vector, we take this smoothing approach
  const double delta = 0.05;
  double z0, zfront, zleft;
  z0 = ground_height_func(loc);
  zfront = ground_height_func(chrono::vehicle::ChWorldFrame::FromISO(loc_ISO + chrono::ChVector3d(delta, 0, 0)));
  zleft = ground_height_func(chrono::vehicle::ChWorldFrame::FromISO(loc_ISO + chrono::ChVector3d(0, delta, 0)));
  chrono::ChVector3d p0(loc_ISO.x(), loc_ISO.y(), z0);
  chrono::ChVector3d pfront(loc_ISO.x() + delta, loc_ISO.y(), zfront);
  chrono::ChVector3d pleft(loc_ISO.x(), loc_ISO.y() + delta, zleft);
  chrono::ChVector3d normal_ISO;
  chrono::ChVector3d r1, r2;
  r1 = pfront - p0;
  r2 = pleft - p0;
  normal_ISO = chrono::Vcross(r1, r2);
  if (normal_ISO.z() <= 0.0) {
      std::cerr << "Fatal: wrong surface normal!" << std::endl;
      throw std::runtime_error("Fatal: wrong surface normal!");
  }
  chrono::ChVector3d normal = chrono::vehicle::ChWorldFrame::FromISO(normal_ISO);
  normal.Normalize();

  terrain.height = z0;
  terrain.nx = normal.x();
  terrain.ny = normal.y();
  terrain.nz = normal.z();

  // terrain.height = ground_height_func(x, y);
  // const double h_dx = ground_height_func(x + delta, y);
  // const double h_dy = ground_height_func(x, y + delta);

  // const Eigen::Vector3d P1 = {x, y, terrain.height};
  // const Eigen::Vector3d P2 = {x + delta, y, h_dx};
  // const Eigen::Vector3d P3 = {x, y + delta, h_dy};

  // const Eigen::Vector3d v1 = P2 - P1;
  // const Eigen::Vector3d v2 = P3 - P1;

  // const Eigen::Vector3d normal = v1.cross(v2);
  // const double normal_norm = normal.norm();
  // terrain.nx = normal[0] / normal_norm;
  // terrain.ny = normal[1] / normal_norm;
  // terrain.nz = normal[2] / normal_norm;

  std::cout << "x:\t" << x << ", y:\t" << y << ", h:\t" << terrain.height
            <<  ", nx:\t" << terrain.nx << ", ny:\t" << terrain.ny << ", nz:\t" << terrain.nz << "\n";
}

// Function to set terrain friction coefficient
double GetTerrainFriction() {
  // Constant friction value for all terrain
  return 0.8;
}

// Function to initialize the input array with default terrain parameters
void InitializeInputArray(double input[simulation_interface::Input::LENGTH]) {
  // Clear all inputs
  for (int i = 0; i < simulation_interface::Input::LENGTH; i++) {
    input[i] = 0.0;
  }
  
  // Set default terrain parameters for each wheel
  // Default heights (flat ground at z=0)
  input[simulation_interface::Input::TERRAIN_HEIGHT_FL] = 0.0;
  input[simulation_interface::Input::TERRAIN_HEIGHT_FR] = 0.0;
  input[simulation_interface::Input::TERRAIN_HEIGHT_RL] = 0.0;
  input[simulation_interface::Input::TERRAIN_HEIGHT_RR] = 0.0;
  
  // Default normals (pointing up)
  input[simulation_interface::Input::TERRAIN_NORMAL_X_FL] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Y_FL] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Z_FL] = 1.0;
  
  input[simulation_interface::Input::TERRAIN_NORMAL_X_FR] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Y_FR] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Z_FR] = 1.0;
  
  input[simulation_interface::Input::TERRAIN_NORMAL_X_RL] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Y_RL] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Z_RL] = 1.0;
  
  input[simulation_interface::Input::TERRAIN_NORMAL_X_RR] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Y_RR] = 0.0;
  input[simulation_interface::Input::TERRAIN_NORMAL_Z_RR] = 1.0;
  
  // Default friction coefficients
  double friction = GetTerrainFriction();
  input[simulation_interface::Input::TERRAIN_MU_FL] = friction;
  input[simulation_interface::Input::TERRAIN_MU_FR] = friction;
  input[simulation_interface::Input::TERRAIN_MU_RL] = friction;
  input[simulation_interface::Input::TERRAIN_MU_RR] = friction;
}

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

  // Initialize input array with default values
  double input[simulation_interface::Input::LENGTH];
  InitializeInputArray(input);
  
  double output[simulation_interface::Output::LENGTH];
  
  // Simulation time and time step
  double time = 0.0;
  const double time_step = 0.01; // 10ms

  // ground_height_func_t ground_height_func = CalculateFlatGroundHeight;
  // ground_height_func_t ground_height_func = CalculateRadialWaveGroundHeight;
  ground_height_func_t ground_height_func = CalculateRampUpGroundHeight;

  Terrain fl_terrain;
  Terrain fr_terrain;
  Terrain rl_terrain;
  Terrain rr_terrain;
  
  int frame_count = 0;
  
  while (vis->Run()) {
    // Update time
    time += time_step;
    
    // First step to get wheel positions from output
    simulation_interface.Step(input, output);
    
    // Extract wheel positions from the output array
    double fl_x = output[simulation_interface::Output::QUERY_POINT_X_FL];
    double fl_y = output[simulation_interface::Output::QUERY_POINT_Y_FL];
    
    double fr_x = output[simulation_interface::Output::QUERY_POINT_X_FR];
    double fr_y = output[simulation_interface::Output::QUERY_POINT_Y_FR];
    
    double rl_x = output[simulation_interface::Output::QUERY_POINT_X_RL];
    double rl_y = output[simulation_interface::Output::QUERY_POINT_Y_RL];
    
    double rr_x = output[simulation_interface::Output::QUERY_POINT_X_RR];
    double rr_y = output[simulation_interface::Output::QUERY_POINT_Y_RR];
    
    // Calculate terrain at each wheel position
    FillTerrain(fl_x, fl_y, ground_height_func, fl_terrain);
    FillTerrain(fr_x, fr_y, ground_height_func, fr_terrain);
    FillTerrain(rl_x, rl_y, ground_height_func, rl_terrain);
    FillTerrain(rr_x, rr_y, ground_height_func, rr_terrain);

    input[simulation_interface::Input::TERRAIN_HEIGHT_FL] = fl_terrain.height;
    input[simulation_interface::Input::TERRAIN_HEIGHT_FR] = fr_terrain.height;
    input[simulation_interface::Input::TERRAIN_HEIGHT_RL] = rl_terrain.height;
    input[simulation_interface::Input::TERRAIN_HEIGHT_RR] = rr_terrain.height;

    input[simulation_interface::Input::TERRAIN_NORMAL_X_FL] = fl_terrain.nx;
    input[simulation_interface::Input::TERRAIN_NORMAL_Y_FL] = fl_terrain.ny;
    input[simulation_interface::Input::TERRAIN_NORMAL_Z_FL] = fl_terrain.nz;

    input[simulation_interface::Input::TERRAIN_NORMAL_X_FR] = fr_terrain.nx;
    input[simulation_interface::Input::TERRAIN_NORMAL_Y_FR] = fr_terrain.ny;
    input[simulation_interface::Input::TERRAIN_NORMAL_Z_FR] = fr_terrain.nz;
    
    input[simulation_interface::Input::TERRAIN_NORMAL_X_RL] = rl_terrain.nx;
    input[simulation_interface::Input::TERRAIN_NORMAL_Y_RL] = rl_terrain.ny;
    input[simulation_interface::Input::TERRAIN_NORMAL_Z_RL] = rl_terrain.nz;
    
    input[simulation_interface::Input::TERRAIN_NORMAL_X_RR] = rr_terrain.nx;
    input[simulation_interface::Input::TERRAIN_NORMAL_Y_RR] = rr_terrain.ny;
    input[simulation_interface::Input::TERRAIN_NORMAL_Z_RR] = rr_terrain.nz;
    
    // Set constant friction coefficient for all wheels
    double friction = GetTerrainFriction();
    input[simulation_interface::Input::TERRAIN_MU_FL] = friction;
    input[simulation_interface::Input::TERRAIN_MU_FR] = friction;
    input[simulation_interface::Input::TERRAIN_MU_RL] = friction;
    input[simulation_interface::Input::TERRAIN_MU_RR] = friction;
    
    // Execute simulation step with updated terrain parameters
    simulation_interface.Step(input, output);
  }
  
  return 0;
}
