#include "stiremod.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

void write_csv(const std::string& filename, const std::vector<std::string>& headers, const std::vector<std::vector<double>>& data) {
    std::ofstream file(filename);

    for (size_t i = 0; i < headers.size(); ++i) {
        file << headers[i] << (i < headers.size() - 1 ? "," : "");
    }
    file << "\n";

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i] << (i < row.size() - 1 ? "," : "");
        }
        file << "\n";
    }
    file.close();
    std::cout << "Generated " << filename << std::endl;
}

int main() {
    // PARAMETERS
    // Representative placeholders for a P205/65R15 tire (Ford Expedition data)
    StiremodParams params;

    // Geometry & Load
    params.Fzt = 2403.0;       // Rated load (lbs)
    params.Tw = 10.4;          // Tread width (inches)
    params.Tp = 35.0;          // Tire Pressure (psi)

    // Peak Friction vs Load (Eq 13)
    params.B1x = -4.7226e-4; params.B3x = 1.2688; params.B4x = 2.879e-7;
    params.B1y = -1.9037e-4; params.B3y = 1.1947; params.B4y = 2.2025e-8;

    // Stiffness vs Load (Eq 4, 5)
    params.A0 = -1063.4; params.A1 = 21.169; params.A2 = 4794.7;
    params.Kx = 0.3014;
    params.CS_FZ = 17.7325;    // Longitudinal stiffness slope (normalized)

    // Camber Stiffness (Eq 8)
    params.A3 = 0.87326; params.A4 = 29825.0; params.K_gamma = 0.9;

    // Saturation Shape (Eq 6) - C parameters
    params.C1 = 0.6633; params.C2 = 0.2184; params.C3 = 0.4867;
    params.C4 = 0.1622; params.C5 = 1.2732;

    // Friction Decay (Eq 12)
    params.K_mux = 0.303;

    // Dynamic K_muy parameters (Replacing static K_muy)
    params.K_muy_offset = 0.45091;
    params.K_muy_slope = -7.9655e-5;
    params.K_muy_max_Fz = 1000.0;

    // Misc
    params.Ka = 0.0365;        // Patch length sensitivity (Eq 2)
    params.K1 = -1.1993e-4;    // Aligning stiffness slope (Eq 11)
    params.G1 = 1.3139;        // Aligning moment shape (Eq 10)
    params.G2 = 1.0;           // Combined slip moment factor (Eq 10)

    // Environmental
    params.SN_o = 85.0;
    params.SN_t = 85.0;

    double fz_test = 935.0; // Test load (lbs)

    // 1. Sweep Alpha (for Fy and Mz)
    std::vector<std::vector<double>> data_alpha;
    for (int i = 0; i < 100; ++i) {
        double alpha = -20.0 + (40.0 * i / 99.0); // -20 to 20
        auto forces = stiremod_calculate(params, fz_test, alpha, 0.0, 0.0);
        data_alpha.push_back({alpha, forces.Fy, forces.Mz});
    }
    write_csv("tire_data_alpha.csv", {"Alpha_deg", "Fy_lbs", "Mz_ftlbs"}, data_alpha);

    // 2. Sweep Slip Ratio (for Fx)
    std::vector<std::vector<double>> data_slip;
    for (int i = 0; i < 100; ++i) {
        double slip = -1.0 + (2.0 * i / 99.0); // -1.0 to 1.0
        auto forces = stiremod_calculate(params, fz_test, 0.0, slip, 0.0);
        data_slip.push_back({slip, forces.Fx});
    }
    write_csv("tire_data_slip.csv", {"Slip_Ratio", "Fx_lbs"}, data_slip);

    // 3. Friction Ellipse (Sweep Alpha at different Slip Ratios)
    std::vector<std::vector<double>> data_ellipse;
    std::vector<double> slip_conditions = {0.0, 0.2, 0.4, 0.8};

    for (size_t s_idx = 0; s_idx < slip_conditions.size(); ++s_idx) {
        double s = slip_conditions[s_idx];
        for (int i = 0; i < 60; ++i) {
            double alpha = 0.0 + (90.0 * i / 59.0); // 0 to 90 deg
            auto forces = stiremod_calculate(params, fz_test, alpha, s, 0.0);
            data_ellipse.push_back({(double)s_idx, s, alpha, forces.Fx, forces.Fy});
        }
    }
    write_csv("tire_data_ellipse.csv", {"Slip_ID", "Slip_Val", "Alpha_deg", "Fx_lbs", "Fy_lbs"}, data_ellipse);

    return 0;
}