#include "salaani_model.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include <string>

namespace {
  // Test each tire
  struct TireTest {
    const char* name;
    const tire::Params& params;
  };

  TireTest tires[] = {
      {"Bridgestone P255/35R18", tire::TireData::bridgestone_255_35R18},
      {"Bridgestone P225/40R18", tire::TireData::bridgestone_225_40R18},
      {"Continental P265/70R17", tire::TireData::continental_265_70R17},
      {"Goodyear P225/60R16", tire::TireData::goodyear_225_60R16}
  };
}

namespace tire {

// Helper function to calculate peak friction coefficients
std::pair<double, double> calculatePeakFriction(const tire::Params& tire, double FZ) {
  double MURATIO = 1.0;
  double FZ1 = (FZ < tire.FZ0) ? tire.FZ0 : FZ;
  
  double MUXp = MURATIO * tire.mu_p0_long * std::pow(FZ1/tire.FZ0, tire.eta2_long + tire.eta1_long * std::log(FZ1/tire.FZ0));
  
  double MUYp = MURATIO * tire.mu_p0_lat * std::pow(FZ1/tire.FZ0, tire.eta2_lat + tire.eta1_lat * std::log(FZ1/tire.FZ0));
  
  return {MUXp, MUYp};
}

// Helper function to calculate lateral stiffness C_alpha
double calculateLateralStiffness(const tire::Params& tire, double FZ) {
  double CA = tire.Cam * (1.0 - std::exp(tire.C1 * std::pow(FZ/tire.FZCam, 2) + 
                                        tire.C2 * (FZ/tire.FZCam)));
  return CA;
}

// Generic function to generate a CSV file
void generateCSV(const std::string& filename, const std::string& header, const std::vector<std::vector<double>>& data) {
  std::ofstream file(filename);
  if (!file.is_open()) {
      std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
      return;
  }

  file << header << std::endl;
  // Assuming data[0] holds the x-axis and subsequent vectors hold y-axis values for each series.
  // Transpose the data for CSV output where each row is a data point.
  if (!data.empty() && !data[0].empty()) {
      for (size_t i = 0; i < data[0].size(); ++i) {
          for (size_t j = 0; j < data.size(); ++j) {
              file << data[j][i];
              if (j < data.size() - 1) {
                  file << ",";
              }
          }
          file << std::endl;
      }
  }
  file.close();
  std::cout << "Successfully generated " << filename << std::endl;
}

// Function to replace spaces and slashes in tire names for filenames
std::string cleanTireName(const std::string& name) {
  std::string cleaned_name = name;
  std::replace(cleaned_name.begin(), cleaned_name.end(), ' ', '_');
  std::replace(cleaned_name.begin(), cleaned_name.end(), '/', '_');
  return cleaned_name;
}

void exportTireModelToCsv() {
  std::cout << "=== Salaani Tire Model CSV Export (By Tire Type and Sampled FZ) ===" << std::endl;
  std::cout << std::endl;
  
  // Define the normal forces for sampling
  std::vector<double> normal_forces_to_sample = {402.0, 802.0, 1202.0, 1602.0, 2001.0, 6000.0};
  
  double camber_angle = 0.0;     // No camber
  
  std::cout << "Generating CSV files for each tire type..." << std::endl;
  std::cout << std::endl;
  
  // Loop through each tire type
  for (const auto& tire_test : tires) {
      std::string cleaned_tire_name = "salaani_tire_results/" + cleanTireName(tire_test.name);
      std::cout << "--- Generating CSVs for " << tire_test.name << " ---" << std::endl;

      // 1. Lateral Force vs Slip Angle
      {
          std::string filename = cleaned_tire_name + "_lateral_force_vs_slip_angle.csv";
          std::string header = "Slip_Angle_deg";
          for (double fz_val : normal_forces_to_sample) {
              header += ",FY_FZ_" + std::to_string(static_cast<int>(fz_val));
          }

          std::vector<std::vector<double>> data;
          std::vector<double> slip_angles_deg;
          for (double alpha_deg = -30.0; alpha_deg <= 30.0; alpha_deg += 0.5) {
              slip_angles_deg.push_back(alpha_deg);
          }
          data.push_back(slip_angles_deg); // First column is slip angle

          for (double fz_val : normal_forces_to_sample) {
              std::vector<double> fy_values;
              for (double alpha_deg : slip_angles_deg) {
                  double alpha_rad = alpha_deg * 3.14 / 180.0;
                  tire::SalaaniTireModel tire_model(tire_test.params);
                  Forces forces = tire_model.calculateTireForces(
                      alpha_rad, 0.0, camber_angle, fz_val);
                  fy_values.push_back(forces.FY);
              }
              data.push_back(fy_values); // Add FY data for this FZ
          }
          generateCSV(filename, header, data);
      }
      
      // 2. Aligning Moment vs Slip Angle
      {
          std::string filename = cleaned_tire_name + "_aligning_moment_vs_slip_angle.csv";
          std::string header = "Slip_Angle_deg";
          for (double fz_val : normal_forces_to_sample) {
              header += ",MZ_FZ_" + std::to_string(static_cast<int>(fz_val));
          }

          std::vector<std::vector<double>> data;
          std::vector<double> slip_angles_deg;
          for (double alpha_deg = -15.0; alpha_deg <= 15.0; alpha_deg += 0.5) {
              slip_angles_deg.push_back(alpha_deg);
          }
          data.push_back(slip_angles_deg); // First column is slip angle

          for (double fz_val : normal_forces_to_sample) {
              std::vector<double> mz_values;
              for (double alpha_deg : slip_angles_deg) {
                  double alpha_rad = alpha_deg * 3.14 / 180.0;
                  tire::SalaaniTireModel tire_model(tire_test.params);
                  Forces forces = tire_model.calculateTireForces(
                      alpha_rad, 0.0, camber_angle, fz_val);
                  mz_values.push_back(forces.MZ);
              }
              data.push_back(mz_values); // Add MZ data for this FZ
          }
          generateCSV(filename, header, data);
      }
      
      // 3. Aligning Moment vs Lateral Force (Single CSV with FZ column for this tire)
      {
          std::string filename = cleaned_tire_name + "_aligning_moment_vs_lateral_force.csv";
          std::string header = "Normal_Load_lbs,Lateral_Force_lbs,Aligning_Moment_ft-lbs";
          
          std::ofstream file(filename);
          file << header << std::endl;

          for (double current_fz : normal_forces_to_sample) {
              for (double alpha_deg = -15.0; alpha_deg <= 15.0; alpha_deg += 0.5) {
                  double alpha_rad = alpha_deg * 3.14 / 180.0;
                  tire::SalaaniTireModel tire_model(tire_test.params);
                  Forces forces = tire_model.calculateTireForces(
                      alpha_rad, 0.0, camber_angle, current_fz);
                  file << current_fz << "," << forces.FY << "," << forces.MZ << std::endl;
              }
          }
          file.close();
          std::cout << "Successfully generated " << filename << std::endl;
      }
      
      // 4. Longitudinal Force vs Longitudinal Slip
      {
          std::string filename = cleaned_tire_name + "_longitudinal_force_vs_slip.csv";
          std::string header = "Longitudinal_Slip_pct";
          for (double fz_val : normal_forces_to_sample) {
              header += ",FX_FZ_" + std::to_string(static_cast<int>(fz_val));
          }

          std::vector<std::vector<double>> data;
          std::vector<double> slip_pct_values;
          for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
              slip_pct_values.push_back(slip * 100.0);
          }
          data.push_back(slip_pct_values); // First column is slip percentage

          for (double fz_val : normal_forces_to_sample) {
              std::vector<double> fx_values;
              for (double slip : slip_pct_values) { // Note: slip is percentage here, convert back to ratio for model
                  tire::SalaaniTireModel tire_model(tire_test.params);
                  Forces forces = tire_model.calculateTireForces(
                      0.0, slip / 100.0, camber_angle, fz_val);
                  fx_values.push_back(forces.FX);
              }
              data.push_back(fx_values); // Add FX data for this FZ
          }
          generateCSV(filename, header, data);
      }
      
      // 5. Lateral Force vs Longitudinal Force (Friction Circle - Single CSV with FZ column for this tire)
      {
          std::string filename = cleaned_tire_name + "_friction_circle.csv";
          std::string header = "Normal_Load_lbs,Longitudinal_Force_lbs,Lateral_Force_lbs";
          std::ofstream file(filename);
          file << header << std::endl;

        //   for (double current_fz : normal_forces_to_sample) {
        double current_fz = 1005.0;
              for (double alpha_deg = -8.0; alpha_deg <= 8.0; alpha_deg += 2.0) {
                  double alpha_rad = alpha_deg * 3.14 / 180.0;
                  for (double slip = -0.9; slip <= 0.9; slip += 0.01) {
                      tire::SalaaniTireModel tire_model(tire_test.params);
                      Forces forces = tire_model.calculateTireForces(
                          alpha_rad, slip, camber_angle, current_fz);
                      file << current_fz << "," << forces.FX << "," << forces.FY << std::endl;
                  }
              }
        //   }
          file.close();
          std::cout << "Successfully generated " << filename << std::endl;
      }
      
      // 6. Lateral Force vs Longitudinal Slip (at fixed slip angles)
      {
          std::string filename = cleaned_tire_name + "_lateral_force_vs_longitudinal_slip.csv";
          std::vector<double> slip_angles = {-6.0, -4.0, -2.0, 2.0, 4.0, 6.0}; // degrees

          std::string header = "Longitudinal_Slip_pct";
        //   for (double fz_val : normal_forces_to_sample) {
        double fz_val = 1005.0;
              for (double alpha_deg : slip_angles) {
                  header += ",FY_FZ_" + std::to_string(static_cast<int>(fz_val)) + "_alpha_" + std::to_string(static_cast<int>(alpha_deg)) + "deg";
              }
        //   }
          
          std::ofstream file(filename);
          file << header << std::endl;

          for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
              file << slip * 100.0;
            //   for (double current_fz : normal_forces_to_sample) {
            double current_fz = 1005.0;
                  for (double alpha_deg : slip_angles) {
                      double alpha_rad = alpha_deg * 3.14 / 180.0;
                      tire::SalaaniTireModel tire_model(tire_test.params);
                      Forces forces = tire_model.calculateTireForces(
                          alpha_rad, slip, camber_angle, current_fz);
                      file << "," << forces.FY;
                  }
            //   }
              file << std::endl;
          }
          file.close();
          std::cout << "Successfully generated " << filename << std::endl;
      }
      
      // 7. Longitudinal Peak Coefficient of Friction vs Normal Load (sampled FZ)
      {
          std::string filename = cleaned_tire_name + "_longitudinal_peak_friction_vs_load.csv";
          std::string header = "Normal_Load_lbs,MUXp";

          std::vector<std::vector<double>> data;
          data.resize(2); // For Normal_Load and MUXp

          for (double fz = 0.0; fz <= 1500.0; fz += 100) {
              data[0].push_back(fz);
              auto friction = calculatePeakFriction(tire_test.params, fz);
              data[1].push_back(friction.first); // Longitudinal peak friction
          }
          generateCSV(filename, header, data);
      }
      
      // 8. Longitudinal Force vs Longitudinal Slip (Combined Slips)
      {
          std::string filename = cleaned_tire_name + "_longitudinal_force_combined_slip.csv";
          std::vector<double> slip_angles = {0.0, 2.0, 4.0, 6.0}; // degrees
          
          std::string header = "Longitudinal_Slip_pct";
          for (double fz_val : normal_forces_to_sample) {
              for (double alpha_deg : slip_angles) {
                  header += ",FX_FZ_" + std::to_string(static_cast<int>(fz_val)) + "_alpha_" + std::to_string(static_cast<int>(alpha_deg)) + "deg";
              }
          }

          std::ofstream file(filename);
          file << header << std::endl;
          
          for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
              file << slip * 100.0;
              for (double current_fz : normal_forces_to_sample) {
                  for (double alpha_deg : slip_angles) {
                      double alpha_rad = alpha_deg * 3.14 / 180.0;
                      tire::SalaaniTireModel tire_model(tire_test.params);
                      Forces forces = tire_model.calculateTireForces(
                          alpha_rad, slip, camber_angle, current_fz);
                      file << "," << forces.FX;
                  }
              }
              file << std::endl;
          }
          file.close();
          std::cout << "Successfully generated " << filename << std::endl;
      }
      
      // 9. Lateral Peak Friction vs Longitudinal Peak Friction (Single CSV with FZ column for this tire)
      {
          std::string filename = cleaned_tire_name + "_lateral_vs_longitudinal_peak_friction.csv";
          std::string header = "Normal_Load_lbs,MUXp,MUYp";
          std::ofstream file(filename);
          file << header << std::endl;
          
          for (double fz : normal_forces_to_sample) {
              auto friction = calculatePeakFriction(tire_test.params, fz);
              file << fz << "," << friction.first << "," << friction.second << std::endl;
          }
          file.close();
          std::cout << "Successfully generated " << filename << std::endl;
      }
      
      // 10. C_alpha (Lateral Stiffness) vs Normal Load (sampled FZ)
      {
          std::string filename = cleaned_tire_name + "_lateral_stiffness_vs_load.csv";
          std::string header = "Normal_Load_lbs,C_alpha";

          std::vector<std::vector<double>> data;
          data.resize(2); // For Normal_Load and C_alpha

          for (double fz : normal_forces_to_sample) {
              data[0].push_back(fz);
              double CA = calculateLateralStiffness(tire_test.params, fz);
              data[1].push_back(CA / (180.0 / acos(-1.0))); // Convert from lbs/rad to lbs/deg
          }
          generateCSV(filename, header, data);
      }
      std::cout << std::endl; // Separator for clarity between tire types
  }
  
  // Summary output for basic verification
  std::cout << "\n=== Basic Model Verification (at FZ = " << normal_forces_to_sample[0] << " lbs, for all tires) ===" << std::endl;
  double test_alpha = 5.0 * 3.14 / 180.0;
  double test_slip = 0.1;
  double verification_fz = normal_forces_to_sample[0]; // Use the first sampled FZ for verification summary
  
  for (const auto& tire_test : tires) {
    try {
      tire::SalaaniTireModel tire_model(tire_test.params);
      Forces result = tire_model.calculateTireForces(
          test_alpha, test_slip, camber_angle, verification_fz);
      
      std::cout << tire_test.name << ":" << std::endl;
      std::cout << "  FX = " << std::setw(8) << std::fixed << std::setprecision(1) << result.FX << " lbs" << std::endl;
      std::cout << "  FY = " << std::setw(8) << std::fixed << std::setprecision(1) << result.FY << " lbs" << std::endl;
      std::cout << "  MZ = " << std::setw(8) << std::fixed << std::setprecision(3) << result.MZ << " ft-lbs" << std::endl;
      std::cout << "  MX = " << std::setw(8) << std::fixed << std::setprecision(3) << result.MX << " ft-lbs" << std::endl;
      std::cout << std::endl;
      } catch (const std::exception& e) {
          std::cout << "  ERROR: " << e.what() << std::endl;
          std::cout << std::endl;
      }
  }
}

// Example usage and demonstration
void demonstrateTireModel() {
  std::cout << "=== Salaani Tire Model Demonstration ===" << std::endl;
  std::cout << std::endl;
  
  // Test conditions
  double slip_angle = 5.0 * 3.14 / 180.0;  // 5 degrees in radians
  double longitudinal_slip = 0.1;           // 10% slip
  double camber_angle = 0.0;                // 0 degrees
  double normal_force = 1000.0;             // 1000 lbs
  
  std::cout << "Test Conditions:" << std::endl;
  std::cout << "  Slip Angle: " << slip_angle * 180.0 / 3.14 << " degrees" << std::endl;
  std::cout << "  Longitudinal Slip: " << longitudinal_slip * 100.0 << "%" << std::endl;
  std::cout << "  Camber Angle: " << camber_angle * 180.0 / 3.14 << " degrees" << std::endl;
  std::cout << "  Normal Force: " << normal_force << " lbs" << std::endl;
  std::cout << std::endl;
  
  for (const auto& tire_test : tires) {
      std::cout << "=== " << tire_test.name << " ===" << std::endl;

      try {
          tire::SalaaniTireModel tire_model(tire_test.params);
          Forces result = tire_model.calculateTireForces(
              slip_angle, longitudinal_slip, camber_angle, normal_force);

          std::cout << "  Longitudinal Force (FX): " << result.FX << " lbs" << std::endl;
          std::cout << "  Lateral Force (FY): " << result.FY << " lbs" << std::endl;
          std::cout << "  Aligning Moment (MZ): " << result.MZ << " ft-lbs" << std::endl;
          std::cout << "  Overturning Moment (MX): " << result.MX << " ft-lbs" << std::endl;
          std::cout << std::endl;
      } catch (const std::exception& e) {
          std::cout << "  ERROR: " << e.what() << std::endl;
          std::cout << std::endl;
      }
  }
}

} // namespace tire

int main() {
  tire::demonstrateTireModel();
  tire::exportTireModelToCsv();
  return 0;
}
