#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>

namespace tire {

// Structure to hold tire parameters (based on Appendix B data)
struct TireParameters {
    // Tire lateral stiffness parameters (Equation 26)
    double C1, C2;        // Lateral stiffness coefficients (CA1, CA2 in MATLAB)
    double Cam;           // Maximum lateral stiffness (CAm in MATLAB)
    double FZCam;         // Reference load for lateral stiffness (CAFzm in MATLAB)
    
    // Tire longitudinal stiffness parameters (Equation 27)
    double Ckm;           // Initial longitudinal stiffness (CSO in MATLAB)
    double FZCKM;         // Reference load for longitudinal stiffness (FzxO in MATLAB)
    double n_val;         // Longitudinal stiffness exponent (Eta in MATLAB)
    
    // Lateral peak coefficient of friction (Equation 22)
    double eta1_lat;      // Lateral friction coefficient 1 (Muy1 in MATLAB)
    double eta2_lat;      // Lateral friction coefficient 2 (Muy2 in MATLAB)
    double mu_p0_lat;     // Lateral peak friction at reference load (MuyO in MATLAB)
    
    // Longitudinal peak coefficient of friction (Equation 22)
    double eta0_long;     // Longitudinal friction coefficient 0 (Mux2 in MATLAB)
    double eta1_long;     // Longitudinal friction coefficient 1 (Mux1 in MATLAB)
    double mu_p0_long;    // Longitudinal peak friction at reference load (MuxO in MATLAB)
    
    double FZ0;           // Reference load for friction calculations (FzO in MATLAB)
    
    // Lateral decay of friction (Equation 23)
    double d1_lat;        // Lateral decay coefficient 1 (KMUy1 in MATLAB)
    double d2_lat;        // Lateral decay coefficient 2 (KMUy2 in MATLAB)
    double d3_lat;        // Lateral decay coefficient 3 (KMUy3 in MATLAB)
    double epsilon_sy;    // Lateral sliding coefficient (Lsy in MATLAB)
    
    // Longitudinal decay of friction (Equation 23)
    double d1_long;       // Longitudinal decay coefficient 1 (KMUx1 in MATLAB)
    double d2_long;       // Longitudinal decay coefficient 2 (KMUx2 in MATLAB)
    double d3_long;       // Longitudinal decay coefficient 3 (KMUx3 in MATLAB)
    double epsilon_xx;    // Longitudinal sliding coefficient (Lsx in MATLAB)
    
    // Aligning moment pneumatic trail (Equation 29)
    double tz1, tz2;      // Pneumatic trail coefficients
    
    // Aligning moment constants (Equation 20)
    double epsilon_x;     // Sliding force eccentricity (Epsx in MATLAB)
    double m1, m0;        // Aligning moment constants
    
    // Overturning moment arm (Equation 30)
    double tx1, tx2, tx3; // Overturning moment arm coefficients
    
    // Inclination angle lateral force stiffness (Equation 31)
    double Cr1, Cr2;      // Camber stiffness coefficients (GAMMA1, GAMMA2 in MATLAB)
    
    // Additional parameters
    double Rt;            // Contact patch length
    double Cz_contact;    // Contact patch stiffness
    double beta;          // Tire relaxation length
    double plysteer;      // Plysteer offset
    
    // Friction coefficients (assumed nominal = test conditions)
    double MUNOM = 1.0;   // Nominal friction coefficient
    double MUNTEST = 1.0; // Test friction coefficient
};


// Predefined tire parameter sets from Appendix B
namespace TireData {
    
    // Data for Bridgestone P255/35R18
    const TireParameters bridgestone_255_35R18 = {
        // Lateral stiffness parameters
        .C1 = -1.59242290928,
        .C2 = -1.52199874366,
        .Cam = 4.010704565e+4,
        .FZCam = 3500,
        
        // Longitudinal stiffness parameters
        .Ckm = 5.099418257e+3,
        .FZCKM = 1.983217617e+2,
        .n_val = 0.91648575346,
        
        // Lateral peak friction
        .eta1_lat = -0.07042627962,
        .eta2_lat = 2.824063502e-6,
        .mu_p0_lat = 1.208943826166,
        
        // Longitudinal peak friction
        .eta0_long = 0.0,
        .eta1_long = -0.01439837983,
        .mu_p0_long = 1.243813670770,
        .FZ0 = 4.004967669e+2,
        
        // Lateral decay of friction
        .d1_lat = -3.135537996e-7,
        .d2_lat = 9.273982487e-4,
        .d3_lat = -0.07310137784,
        .epsilon_sy = 1.05,
        
        // Longitudinal decay of friction
        .d1_long = -7.106354884e-8,
        .d2_long = 2.640661279e-4,
        .d3_long = 0.124604606202,
        .epsilon_xx = 1.05,
        
        // Aligning moment parameters
        .tz1 = -5.429316539e-9,
        .tz2 = -5.665247043e-6,
        .epsilon_x = 0.01,
        .m1 = 0.6,
        .m0 = M_PI / 4.0,
        
        // Overturning moment parameters
        .tx1 = 1.745817181e-8,
        .tx2 = -9.717871139e-6,
        .tx3 = -0.02295062296,
        
        // Inclination angle parameters
        .Cr1 = 0.763265331360,
        .Cr2 = 3.215889442e-4,
        
        // Additional parameters
        .Rt = 12.25,
        .Cz_contact = 2.043428888e+3,
        .beta = 0.717385585628,
        .plysteer = -1.788346665e-4
    };

    // Data for Bridgestone P225/40R18
    const TireParameters bridgestone_225_40R18 = {
        // Lateral stiffness parameters
        .C1 = -2.87732766366,
        .C2 = -1.58418227108,
        .Cam = 3.036676314e+4,
        .FZCam = 3000,
        
        // Longitudinal stiffness parameters
        .Ckm = 5.033347485e+3,
        .FZCKM = 1.993866867e+2,
        .n_val = 0.978492283025,
        
        // Lateral peak friction
        .eta1_lat = -0.06663103908,
        .eta2_lat = -0.01958200369,
        .mu_p0_lat = 1.215873626204,
        
        // Longitudinal peak friction
        .eta0_long = 0.0,
        .eta1_long = -0.04151186868,
        .mu_p0_long = 1.266284298601,
        .FZ0 = 4.017939206e+2,
        
        // Lateral decay of friction
        .d1_lat = -4.077117020e-7,
        .d2_lat = 0.001060743652,
        .d3_lat = -0.01856988106,
        .epsilon_sy = 1.05,
        
        // Longitudinal decay of friction
        .d1_long = -1.09319546e-7,
        .d2_long = 3.399196132e-4,
        .d3_long = 0.113249576597,
        .epsilon_xx = 1.05,
        
        // Aligning moment parameters
        .tz1 = -8.869752030e-9,
        .tz2 = -6.675581613e-5,
        .epsilon_x = 0.01,
        .m1 = 0.6,
        .m0 = M_PI / 4.0,
        
        // Overturning moment parameters
        .tx1 = 1.84431197e-8,
        .tx2 = -3.518428268e-6,
        .tx3 = -0.01897841071,
        
        // Inclination angle parameters
        .Cr1 = 1.028125956136,
        .Cr2 = 2.336756842e-4,
        
        // Additional parameters
        .Rt = 12.25,
        .Cz_contact = 1.773150981e+3,
        .beta = 0.793800855454,
        .plysteer = -0.00137043289
    };

    // Data for Continental P265/70R17
    const TireParameters continental_265_70R17 = {
        // Lateral stiffness parameters
        .C1 = -6.6858662762207,
        .C2 = -2.390762975025,
        .Cam = 2.2918311805e+4,
        .FZCam = 4000,
        
        // Longitudinal stiffness parameters
        .Ckm = 2.9291631099e+3,
        .FZCKM = 1.158373015412,
        .n_val = 1.308139729802,
        
        // Lateral peak friction
        .eta1_lat = -0.046243350096,
        .eta2_lat = -0.128452487507,
        .mu_p0_lat = 1.1328139729802,
        
        // Longitudinal peak friction
        .eta0_long = 0.033127840588,
        .eta1_long = -0.122260930831,
        .mu_p0_long = 1.2134062679177,
        .FZ0 = 6.0055968210e+2,
        
        // Lateral decay of friction
        .d1_lat = 5.797626815e-8,
        .d2_lat = -3.024992368e-4,
        .d3_lat = 0.689854938838,
        .epsilon_sy = 1.07,
        
        // Longitudinal decay of friction
        .d1_long = 7.1643166552e-8,
        .d2_long = 2.16688774920e-4,
        .d3_long = 0.1352352248580,
        .epsilon_xx = 0.95,
        
        // Aligning moment parameters
        .tz1 = -1.0037205363e-8,
        .tz2 = -6.238569625e-5,
        .epsilon_x = 0.01,
        .m1 = 0.6,
        .m0 = M_PI / 4.0,
        
        // Overturning moment parameters
        .tx1 = 1.82132876509e-8,
        .tx2 = 1.26104579008e-5,
        .tx3 = -0.0345896969384,
        
        // Inclination angle parameters
        .Cr1 = 0.873256746984,
        .Cr2 = -2.9279324497e-5,
        
        // Additional parameters
        .Rt = 15.63,
        .Cz_contact = 1.544432073e+3,
        .beta = 2.118775049325,
        .plysteer = -8.4908907595e-4
    };

    // Data for Goodyear P225/60R16
    const TireParameters goodyear_225_60R16 = {
        // Lateral stiffness parameters
        .C1 = -5.49866853851,
        .C2 = -2.11626207379,
        .Cam = 1.604281826e+4,
        .FZCam = 2500,
        
        // Longitudinal stiffness parameters
        .Ckm = 3.840078061e+3,
        .FZCKM = 1.26085510479,
        .n_val = 1.192842481344,
        
        // Lateral peak friction
        .eta1_lat = -0.04960551335,
        .eta2_lat = -0.14093687309,
        .mu_p0_lat = 1.192842481344,
        
        // Longitudinal peak friction
        .eta0_long = 0.026628558245,
        .eta1_long = -0.02196648837,
        .mu_p0_long = 1.075904559373,
        .FZ0 = 4.607008849e+2,
        
        // Lateral decay of friction
        .d1_lat = -9.131892344e-8,
        .d2_lat = 1.798822501e-4,
        .d3_lat = 0.327952438694,
        .epsilon_sy = 1.10,
        
        // Longitudinal decay of friction
        .d1_long = -1.999969731e-9,
        .d2_long = 2.186713890e-4,
        .d3_long = 0.095162164771,
        .epsilon_xx = 1.05,
        
        // Aligning moment parameters
        .tz1 = -1.446513558e-8,
        .tz2 = -8.488540386e-5,
        .epsilon_x = 0.01,
        .m1 = 0.6,
        .m0 = M_PI / 4.0,
        
        // Overturning moment parameters
        .tx1 = 2.387975474e-8,
        .tx2 = 3.106407447e-5,
        .tx3 = -0.05374567698,
        
        // Inclination angle parameters
        .Cr1 = 0.743808159855,
        .Cr2 = -8.772919630e-5,
        
        // Additional parameters
        .Rt = 13.35,
        .Cz_contact = 1.195960963e+3,
        .beta = 1.922252761367,
        .plysteer = 3.714581147e-4
    };
}

// Example usage and demonstration
void demonstrateTireModel() {
    std::cout << "=== Salaani Tire Model Demonstration with Plotting ===" << std::endl;
    std::cout << std::endl;
    
    // Tire selection for detailed analysis
    struct TireTest {
        const char* name;
        const TireParameters& params;
    };
    
    TireTest tires[] = {
        {"Bridgestone P255/35R18", TireData::bridgestone_255_35R18},
        {"Bridgestone P225/40R18", TireData::bridgestone_225_40R18},
        {"Continental P265/70R17", TireData::continental_265_70R17},
        {"Goodyear P225/60R16", TireData::goodyear_225_60R16}
    };
    
    std::vector<std::string> tire_names;
    for (const auto& tire : tires) {
        tire_names.push_back(tire.name);
    }
    
    std::cout << "Generating plots for tire analysis..." << std::endl;
    std::cout << "Note: Run the generated .gp files with gnuplot to create PNG images" << std::endl;
    std::cout << std::endl;
    
    // Test conditions
    double normal_force = 1000.0;  // Standard test load
    double camber_angle = 0.0;     // No camber
    
    // 1. Lateral Force vs Slip Angle
    {
        PlotData plotData;
        plotData.xlabel = "Slip Angle (degrees)";
        plotData.ylabel = "Lateral Force (lbs)";
        plotData.title = "Lateral Force vs Slip Angle";
        
        for (const auto& tire_test : tires) {
            for (double alpha_deg = -15.0; alpha_deg <= 15.0; alpha_deg += 0.5) {
                double alpha_rad = alpha_deg * M_PI / 180.0;
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, alpha_rad, 0.0, camber_angle, normal_force);
                
                plotData.x.push_back(alpha_deg);
                plotData.y.push_back(forces.FY);
            }
        }
        generatePlot(plotData, "lateral_force_vs_slip_angle", tire_names);
    }
    
    // 2. Aligning Moment vs Slip Angle
    {
        PlotData plotData;
        plotData.xlabel = "Slip Angle (degrees)";
        plotData.ylabel = "Aligning Moment (ft-lbs)";
        plotData.title = "Aligning Moment vs Slip Angle";
        
        for (const auto& tire_test : tires) {
            for (double alpha_deg = -15.0; alpha_deg <= 15.0; alpha_deg += 0.5) {
                double alpha_rad = alpha_deg * M_PI / 180.0;
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, alpha_rad, 0.0, camber_angle, normal_force);
                
                plotData.x.push_back(alpha_deg);
                plotData.y.push_back(forces.MZ);
            }
        }
        generatePlot(plotData, "aligning_moment_vs_slip_angle", tire_names);
    }
    
    // 3. Aligning Moment vs Lateral Force
    {
        PlotData plotData;
        plotData.xlabel = "Lateral Force (lbs)";
        plotData.ylabel = "Aligning Moment (ft-lbs)";
        plotData.title = "Aligning Moment vs Lateral Force";
        
        for (const auto& tire_test : tires) {
            for (double alpha_deg = -15.0; alpha_deg <= 15.0; alpha_deg += 0.5) {
                double alpha_rad = alpha_deg * M_PI / 180.0;
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, alpha_rad, 0.0, camber_angle, normal_force);
                
                plotData.x.push_back(forces.FY);
                plotData.y.push_back(forces.MZ);
            }
        }
        generatePlot(plotData, "aligning_moment_vs_lateral_force", tire_names);
    }
    
    // 4. Longitudinal Force vs Longitudinal Slip
    {
        PlotData plotData;
        plotData.xlabel = "Longitudinal Slip (%)";
        plotData.ylabel = "Longitudinal Force (lbs)";
        plotData.title = "Longitudinal Force vs Longitudinal Slip";
        
        for (const auto& tire_test : tires) {
            for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, 0.0, slip, camber_angle, normal_force);
                
                plotData.x.push_back(slip * 100.0);  // Convert to percentage
                plotData.y.push_back(forces.FX);
            }
        }
        generatePlot(plotData, "longitudinal_force_vs_slip", tire_names);
    }
    
    // 5. Lateral Force vs Longitudinal Force (Friction Circle)
    {
        PlotData plotData;
        plotData.xlabel = "Longitudinal Force (lbs)";
        plotData.ylabel = "Lateral Force (lbs)";
        plotData.title = "Lateral Force vs Longitudinal Force (Friction Circle)";
        
        for (const auto& tire_test : tires) {
            // Vary both slip angle and longitudinal slip
            for (double alpha_deg = 0.0; alpha_deg <= 10.0; alpha_deg += 1.0) {
                double alpha_rad = alpha_deg * M_PI / 180.0;
                for (double slip = -0.6; slip <= 0.6; slip += 0.05) {
                    tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                        tire_test.params, alpha_rad, slip, camber_angle, normal_force);
                    
                    plotData.x.push_back(forces.FX);
                    plotData.y.push_back(forces.FY);
                }
            }
        }
        generatePlot(plotData, "friction_circle", tire_names);
    }
    
    // 6. Lateral Force vs Longitudinal Slip (at fixed slip angles)
    {
        PlotData plotData;
        plotData.xlabel = "Longitudinal Slip (%)";
        plotData.ylabel = "Lateral Force (lbs)";
        plotData.title = "Lateral Force vs Longitudinal Slip (Combined Slip)";
        
        // Use first tire for this detailed analysis
        const auto& tire_test = tires[0];
        std::vector<double> slip_angles = {2.0, 4.0, 6.0}; // degrees
        
        for (double alpha_deg : slip_angles) {
            double alpha_rad = alpha_deg * M_PI / 180.0;
            for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, alpha_rad, slip, camber_angle, normal_force);
                
                plotData.x.push_back(slip * 100.0);
                plotData.y.push_back(forces.FY);
            }
        }
        
        // Create labels for different slip angles
        std::vector<std::string> angle_labels;
        for (double angle : slip_angles) {
            angle_labels.push_back("α = " + std::to_string((int)angle) + "°");
        }
        generatePlot(plotData, "lateral_force_vs_longitudinal_slip", angle_labels);
    }
    
    // 7. Longitudinal Peak Coefficient of Friction vs Normal Load
    {
        PlotData plotData;
        plotData.xlabel = "Normal Load (lbs)";
        plotData.ylabel = "Longitudinal Peak Friction Coefficient";
        plotData.title = "Longitudinal Peak Friction vs Normal Load";
        
        for (const auto& tire_test : tires) {
            for (double fz = 200.0; fz <= 2000.0; fz += 50.0) {
                auto friction = calculatePeakFriction(tire_test.params, fz);
                plotData.x.push_back(fz);
                plotData.y.push_back(friction.first); // Longitudinal peak friction
            }
        }
        generatePlot(plotData, "longitudinal_peak_friction_vs_load", tire_names);
    }
    
    // 8. Longitudinal Force vs Longitudinal Slip (Combined Slips)
    {
        PlotData plotData;
        plotData.xlabel = "Longitudinal Slip (%)";
        plotData.ylabel = "Longitudinal Force (lbs)";
        plotData.title = "Longitudinal Force vs Slip (Combined Conditions)";
        
        // Use first tire with different slip angles
        const auto& tire_test = tires[0];
        std::vector<double> slip_angles = {0.0, 2.0, 4.0, 6.0}; // degrees
        
        for (double alpha_deg : slip_angles) {
            double alpha_rad = alpha_deg * M_PI / 180.0;
            for (double slip = -0.8; slip <= 0.8; slip += 0.02) {
                tire::SalaaniTireModel::Forces forces = tire_model.calculateTireForces(
                    tire_test.params, alpha_rad, slip, camber_angle, normal_force);
                
                plotData.x.push_back(slip * 100.0);
                plotData.y.push_back(forces.FX);
            }
        }
        
        std::vector<std::string> angle_labels;
        for (double angle : slip_angles) {
            angle_labels.push_back("α = " + std::to_string((int)angle) + "°");
        }
        generatePlot(plotData, "longitudinal_force_combined_slip", angle_labels);
    }
    
    // 9. Lateral Peak Friction vs Longitudinal Peak Friction
    {
        PlotData plotData;
        plotData.xlabel = "Longitudinal Peak Friction Coefficient";
        plotData.ylabel = "Lateral Peak Friction Coefficient";
        plotData.title = "Lateral vs Longitudinal Peak Friction";
        
        for (const auto& tire_test : tires) {
            for (double fz = 200.0; fz <= 2000.0; fz += 50.0) {
                auto friction = calculatePeakFriction(tire_test.params, fz);
                plotData.x.push_back(friction.first);  // Longitudinal
                plotData.y.push_back(friction.second); // Lateral
            }
        }
        generatePlot(plotData, "lateral_vs_longitudinal_peak_friction", tire_names);
    }
    
    // 10. C_alpha (Lateral Stiffness) vs Normal Load
    {
        PlotData plotData;
        plotData.xlabel = "Normal Load (lbs)";
        plotData.ylabel = "Lateral Stiffness C_alpha (lbs/rad)";
        plotData.title = "Lateral Stiffness vs Normal Load";
        
        for (const auto& tire_test : tires) {
            for (double fz = 200.0; fz <= 2000.0; fz += 50.0) {
                double CA = calculateLateralStiffness(tire_test.params, fz);
                plotData.x.push_back(fz);
                plotData.y.push_back(CA);
            }
        }
        generatePlot(plotData, "lateral_stiffness_vs_load", tire_names);
    }
    
    std::cout << "All plot files generated successfully!" << std::endl;
    std::cout << "To generate PNG images, run the following commands:" << std::endl;
    std::cout << "  gnuplot lateral_force_vs_slip_angle.gp" << std::endl;
    std::cout << "  gnuplot aligning_moment_vs_slip_angle.gp" << std::endl;
    std::cout << "  gnuplot aligning_moment_vs_lateral_force.gp" << std::endl;
    std::cout << "  gnuplot longitudinal_force_vs_slip.gp" << std::endl;
    std::cout << "  gnuplot friction_circle.gp" << std::endl;
    std::cout << "  gnuplot lateral_force_vs_longitudinal_slip.gp" << std::endl;
    std::cout << "  gnuplot longitudinal_peak_friction_vs_load.gp" << std::endl;
    std::cout << "  gnuplot longitudinal_force_combined_slip.gp" << std::endl;
    std::cout << "  gnuplot lateral_vs_longitudinal_peak_friction.gp" << std::endl;
    std::cout << "  gnuplot lateral_stiffness_vs_load.gp" << std::endl;
    std::cout << std::endl;
    
    // Summary output for basic verification
    std::cout << "=== Basic Model Verification ===" << std::endl;
    double test_alpha = 5.0 * M_PI / 180.0;
    double test_slip = 0.1;
    
    for (const auto& tire_test : tires) {
        tire::SalaaniTireModel::Forces result = tire_model.calculateTireForces(
            tire_test.params, test_alpha, test_slip, camber_angle, normal_force);
        
        std::cout << tire_test.name << ":" << std::endl;
        std::cout << "  FX = " << std::setw(8) << std::fixed << std::setprecision(1) << result.FX << " lbs" << std::endl;
        std::cout << "  FY = " << std::setw(8) << std::fixed << std::setprecision(1) << result.FY << " lbs" << std::endl;
        std::cout << "  MZ = " << std::setw(8) << std::fixed << std::setprecision(3) << result.MZ << " ft-lbs" << std::endl;
        std::cout << "  MX = " << std::setw(8) << std::fixed << std::setprecision(3) << result.MX << " ft-lbs" << std::endl;
        std::cout << std::endl;
    }
}

} // namespace tire

int main() {
    tire::demonstrateTireModel();
    return 0;
}