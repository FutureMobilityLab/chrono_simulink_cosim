#ifndef STIREMOD_H
#define STIREMOD_H

#include <cmath>

struct StiremodParams {
    // Geometry & Load
    double Fzt;       // Rated load (lbs)
    double Tw;        // Tread width (inches)
    double Tp;        // Tire Pressure (psi)

    // Peak Friction vs Load (Eq 13)
    double B1x, B3x, B4x;
    double B1y, B3y, B4y;

    // Stiffness vs Load (Eq 4, 5)
    double A0, A1, A2;
    double Kx;        // Coefficient for cornering stiffness dependence on longitudinal force
    double CS_FZ;     // Longitudinal stiffness slope (normalized)

    // Camber Stiffness (Eq 8)
    double A3, A4;
    double K_gamma;

    // Saturation Shape (Eq 6)
    double C1, C2, C3, C4, C5;

    // Friction Decay (Eq 12)
    double K_mux;
    
    // Dynamic K_muy parameters
    double K_muy_offset;
    double K_muy_slope;
    double K_muy_max_Fz;

    // Misc
    double Ka;        // Patch length sensitivity (Eq 2)
    double K1;        // Aligning stiffness slope (Eq 11)
    double G1, G2;    // Aligning moment shape (Eq 10)

    // Environmental
    double SN_o;      // Skid number of surface
    double SN_t;      // Skid number of test
};

class StiremodTire {
public:
    // Initialize the STIREMOD tire model with a struct of parameters.
    StiremodTire(const StiremodParams& params);

    struct TireForces {
        double Fx;
        double Fy;
        double Mz;
        double alpha_rad;
    };

    /**
     * Calculate tire forces and moments.
     * * Inputs:
     * Fz: Normal load (lbs)
     * alpha_deg: Slip angle (degrees)
     * slip_ratio: Longitudinal slip ratio (S)
     * gamma_deg: Camber angle (degrees)
     * V_mph: Vehicle speed (mph) - used for lag, defaults to 40 for static plots
     * * Returns:
     * TireForces struct containing Fx, Fy, Mz, and alpha_rad
     */
    TireForces calculate(double Fz, double alpha_deg, double slip_ratio, double gamma_deg, double V_mph = 40.0);

private:
    StiremodParams p;
};

#endif // STIREMOD_H