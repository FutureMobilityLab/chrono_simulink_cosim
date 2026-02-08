// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of the single-wheel tire test rig.
//
// =============================================================================

#include <algorithm>

#include "utils/utils.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "demos/SetChronoSolver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;

namespace {
constexpr double LBF_TO_N = 4.4482216153;
constexpr double FTLBF_TO_NM = 1.3558179483;
}

// -----------------------------------------------------------------------------

class SineOffsetFunction : public ChFunctionSine {
  private:
    double m_offset;
  public:
    SineOffsetFunction() : m_offset(0.0) {}
    SineOffsetFunction(double offset, double ampl, double freq, double phase = 0)
        : ChFunctionSine(ampl, freq, phase), m_offset(offset) {}
    virtual ~SineOffsetFunction() {}
    virtual double GetVal(double x) const override {
        return ChFunctionSine::GetVal(x) + m_offset;
    }
    virtual SineOffsetFunction* Clone() const override { return new SineOffsetFunction(*this); }
};

// Contact formulation type (SMC or NSC)
ChContactMethod contact_method = ChContactMethod::NSC;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire model
enum class TireType { RIGID, TMEASY, FIALA, PAC89, PAC02, ANCF4, ANCF8, ANCF_TOROIDAL, REISSNER };
TireType tire_type = TireType::TMEASY;

// Terrain type (RIGID or SCM)
enum class TerrainType { RIGID, SCM };
TerrainType terrain_type = TerrainType::RIGID;

// Read from JSON specification file?
bool use_JSON = true;

bool gnuplot_output = false;
bool blender_output = false;

// -----------------------------------------------------------------------------

int main() {
    chrono::vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
    // Create wheel and tire subsystems
    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");

    std::shared_ptr<ChTire> tire;
    /*
    if (tire_type == TireType::ANCF_TOROIDAL) {
        auto ancf_tire = chrono_types::make_shared<ANCFToroidalTire>("ANCFtoroidal tire");
        ancf_tire->SetRimRadius(0.27);
        ancf_tire->SetHeight(0.18);
        ancf_tire->SetThickness(0.015);
        ancf_tire->SetDivCircumference(40);
        ancf_tire->SetDivWidth(8);
        ancf_tire->SetPressure(320e3);
        ancf_tire->SetAlpha(0.15);
        if (terrain_type == TerrainType::SCM)
            ancf_tire->SetContactSurfaceType(ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH);
        tire = ancf_tire;
    } else if (use_JSON) {
        std::string tire_file;
        switch (tire_type) {
            case TireType::RIGID:
                tire_file = "hmmwv/tire/HMMWV_RigidTire.json";
                break;
            case TireType::TMEASY:
                tire_file = "hmmwv/tire/HMMWV_TMeasyTire.json";
                break;
            case TireType::FIALA:
                tire_file = "hmmwv/tire/HMMWV_FialaTire.json";
                break;
            case TireType::PAC89:
                tire_file = "hmmwv/tire/HMMWV_Pac89Tire.json";
                break;
            case TireType::PAC02:
                tire_file = "hmmwv/tire/HMMWV_Pac02Tire.json";
                break;
            case TireType::ANCF4:
                tire_file = "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
                break;
            case TireType::ANCF8:
                tire_file = "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json";
                break;
            case TireType::REISSNER:
                tire_file = "hmmwv/tire/HMMWV_ReissnerTire.json";
                break;
        }
        tire = ReadTireJSON(vehicle::GetDataFile(tire_file));
    } else {
        switch (tire_type) {
            case TireType::RIGID:
                tire = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");
                break;
            case TireType::TMEASY:
                tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
                break;
            case TireType::FIALA:
                tire = chrono_types::make_shared<hmmwv::HMMWV_FialaTire>("Fiala tire");
                break;
            case TireType::PAC89:
                tire = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("Pac89 tire");
                break;
            case TireType::ANCF4: {
                auto hmmwv_tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>(
                    "ANCF tire", hmmwv::HMMWV_ANCFTire::ElementType::ANCF_4);
                if (terrain_type == TerrainType::SCM)
                    hmmwv_tire->SetContactSurfaceType(ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH);
                tire = hmmwv_tire;
                break;
            }
            case TireType::ANCF8: {
                auto hmmwv_tire = chrono_types::make_shared<hmmwv::HMMWV_ANCFTire>(
                    "ANCF tire", hmmwv::HMMWV_ANCFTire::ElementType::ANCF_8);
                if (terrain_type == TerrainType::SCM)
                    hmmwv_tire->SetContactSurfaceType(ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH);
                tire = hmmwv_tire;
                break;
            }
            case TireType::REISSNER: {
                auto hmmwv_tire = chrono_types::make_shared<hmmwv::HMMWV_ReissnerTire>("Reissner tire");
                if (terrain_type == TerrainType::SCM)
                    hmmwv_tire->SetContactSurfaceType(ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH);
                tire = hmmwv_tire;
                break;
            }
        }
    }
    */

    // This tire model works.
    // const std::string tire_file = "sedan_force/tire/Sedan_Pac89Tire.json";
    // These tire models do not work.
    const std::string tire_file = "sedan_force/tire/continental_P265_70R17.json";
    // const std::string tire_file = "sedan_force/tire/bridgestone_P255_35R18.json";
    tire = vehicle::ReadCustomTireJSON(chrono::vehicle::GetDataFile(tire_file));

    // Create system and set solver
    ChSystem* sys = nullptr;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;
    double step_size = 1e-3;

    if (tire_type == TireType::ANCF4 || tire_type == TireType::ANCF8 || tire_type == TireType::ANCF_TOROIDAL ||
        tire_type == TireType::REISSNER) {
        if (contact_method != ChContactMethod::SMC)
            std::cout << "\nWarning! Contact formulation changed to SMC.\n" << std::endl;
        contact_method = ChContactMethod::SMC;
    }

    switch (contact_method) {
        case ChContactMethod::SMC:
            sys = new ChSystemSMC;
            step_size = 2e-4;
            solver_type = ChSolver::Type::PARDISO_MKL;
            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
            std::static_pointer_cast<ChDeformableTire>(tire)->SetContactFaceThickness(0.02);
            break;

        case ChContactMethod::NSC:
            sys = new ChSystemNSC;
            step_size = 1e-3;
            solver_type = ChSolver::Type::BARZILAIBORWEIN;
            integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
            break;
    }

    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create and configure test rig
    ChTireTestRig rig(wheel, tire, sys);

    ////rig.SetGravitationalAcceleration(0);
    rig.SetNormalLoad(934 * LBF_TO_N);

    ////rig.SetCamberAngle(+15 * CH_DEG_TO_RAD);

    rig.SetTireStepsize(step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::SINGLE_POINT);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    if (terrain_type == TerrainType::RIGID)
        rig.SetTerrainRigid(0.8, 0, 2e7, 100.0);
    else
        rig.SetTerrainSCM(6259.1e3, 5085.6e3, 1.42, 1.58e3, 34.1, 22.17e-3);

    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    // rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(2.0));
    // rig.Initialize();

    // Scenario: pulled wheel
    // rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    //   longitudinal speed: 0.2 m/s
    //   angular speed: 10 RPM
    //   slip angle: sinusoidal +- 5 deg with 5 s period
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));
    // rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionSine>(0.0 * CH_RPM_TO_RAD_S, 1.0));
    rig.SetAngSpeedFunction(chrono_types::make_shared<SineOffsetFunction>(3.5, 10 * CH_RPM_TO_RAD_S, 0.1));
    // rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(30.0 * CH_RPM_TO_RAD_S));
    // rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(0.1 * CH_DEG_TO_RAD, 0.2));
    rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(35 * CH_DEG_TO_RAD, 0.1));

    // Scenario: specified longitudinal slip (overrrides other definitons of motion functions)
    // rig.SetConstantLongitudinalSlip(0.0, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(1.0);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Optionally, modify tire visualization (can be done only after initialization)
    if (auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire)) {
        if (tire_def->GetMeshVisualization())
            tire_def->GetMeshVisualization()->SetColorscaleMinMax(0.0, 5.0);  // range for nodal speed norm
    }

    // Initialize output
    const std::string out_dir = GetChronoOutputPath() + "TIRE_TEST_RIG";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 600);
            vis_irr->SetWindowTitle("Tire Test Rig");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_irr->AddLightDirectional();

            vis_irr->GetActiveCamera()->setFOV(irr::core::PI / 4.5f);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowSize(1200, 600);
            vis_vsg->SetWindowTitle("Tire Test Rig");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.5, 1.0));
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Open CSV file for data output
    std::string output_csv = out_dir + "/tire_test_data.csv";
    std::ofstream csv_output(output_csv);
    std::cout << "Output CSV Path: " << output_csv << std::endl;
    csv_output << "time,long_slip,slip_angle,camber_angle,"
               << "force_x,force_y,force_z,"
               << "point_x,point_y,point_z,"
               << "moment_x,moment_y,moment_z,"
               << "tire_frame_pos_x,tire_frame_pos_y,tire_frame_pos_z,"
               << "tire_frame_rot_x,tire_frame_rot_y,tire_frame_rot_z,"
               << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
               << "linear_velocity_x,linear_velocity_y,linear_velocity_z\n";

    while (vis->Run()) {
        double time = sys->GetChTime();

        auto& loc = rig.GetPos();
        vis->UpdateCamera(loc + ChVector3d(1.0, 2.5, 0.5), loc + ChVector3d(0, 0.25, -0.25));

        vis->BeginScene();
        vis->Render();
        rig.Advance(step_size);
        vis->EndScene();

        // Write data to CSV files
        double time_offset = 0;
        if (time > time_offset) {
            // Get tire forces in local coordinates
            ChCoordsys<> tire_frame;
            auto tforce = tire->ReportTireForceLocal(rig.GetTerrain().get(), tire_frame);

            // Write all data to single CSV file
            auto tire_rot_angles = tire_frame.rot.GetCardanAnglesXYZ();
            csv_output << time << ","
                      << tire->GetLongitudinalSlip() << ","
                      << tire->GetSlipAngle() * CH_RAD_TO_DEG << ","
                      << tire->GetCamberAngle() * CH_RAD_TO_DEG << ","
                      << tforce.force.x() << ","
                    //   << rig.GetDBP() << ","
                      << tforce.force.y() << ","
                      << tforce.force.z() << ","
                      << tforce.point.x() << ","
                      << tforce.point.y() << ","
                      << tforce.point.z() << ","
                      << tforce.moment.x() << ","
                      << tforce.moment.y() << ","
                    //   << tforce.moment.z() << "\n";
                      << tforce.moment.z() << ","
                      << tire_frame.pos.x() << ","
                      << tire_frame.pos.y() << ","
                      << tire_frame.pos.z() << ","
                      << tire_rot_angles.x() * CH_RAD_TO_DEG << ","
                      << tire_rot_angles.y() * CH_RAD_TO_DEG << ","
                      << tire_rot_angles.z() * CH_RAD_TO_DEG << ","
                      << wheel->GetSpindle()->GetAngVelLocal().x() << ","
                      << wheel->GetSpindle()->GetAngVelLocal().y() << ","
                      << wheel->GetSpindle()->GetAngVelLocal().z() << ","
                      << wheel->GetSpindle()->GetLinVel().x() << ","
                      << wheel->GetSpindle()->GetLinVel().y() << ","
                      << wheel->GetSpindle()->GetLinVel().z() << "\n";

            // std::cout << "SlipAngle: " << tire->GetSlipAngle() * CH_RAD_TO_DEG << std::endl;

            // Flush the file periodically to ensure data is written
            if (static_cast<int>(time * 100) % 100 == 0) {
                csv_output.flush();
            }
        }

        // Original debug prints (commented out)
        // std::cout << sys->GetChTime() << std::endl;
        // auto long_slip = tire->GetLongitudinalSlip();
        // auto slip_angle = tire->GetSlipAngle();
        // auto camber_angle = tire->GetCamberAngle();
        // std::cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        // auto tforce = rig.ReportTireForce();
        // auto frc = tforce.force;
        // auto pnt = tforce.point;
        // auto trq = tforce.moment;
        // std::cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        // std::cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        // std::cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }
    // Close CSV file
    csv_output.close();

    std::cout << "\nData has been written to CSV file: " << out_dir << "/tire_test_data.csv" << std::endl;
    std::cout << "File contains:" << std::endl;
    std::cout << "  - Time" << std::endl;
    std::cout << "  - Kinematics (longitudinal slip, slip angle, camber angle)" << std::endl;
    std::cout << "  - Forces (x, y, z components in tire local ISO frame)" << std::endl;
    std::cout << "  - Points (x, y, z components in global frame)" << std::endl;
    std::cout << "  - Moments (x, y, z components in tire local ISO frame)" << std::endl;
    std::cout << "  - Tire frame (position and orientation in global frame)" << std::endl;

    return 0;
}
