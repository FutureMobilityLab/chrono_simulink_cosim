// =============================================================================
// Authors: Trevor Vidano
// =============================================================================
//
// STIREMOD tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "src/tire/StiremodTire.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
StiremodTire::StiremodTire(const std::string& filename)
    : ChStiremodTire(""), m_mass(0), m_normalDamping(0), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

StiremodTire::StiremodTire(const rapidjson::Document& d)
    : ChStiremodTire(""), m_mass(0), m_normalDamping(0), m_has_mesh(false) {
    Create(d);
}

// -----------------------------------------------------------------------------
void StiremodTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_has_vert_table = false;

    m_mass    = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    if (d.HasMember("Coefficient of Friction")) {
        m_mu0 = d["Coefficient of Friction"].GetDouble();
    }

    // -------------------------------------------------------------------------
    // Design Parameters  (SI units)
    // -------------------------------------------------------------------------
    if (d.HasMember("Design Parameters")) {
        const auto& dp = d["Design Parameters"];

        m_unloaded_radius   = dp["Unloaded Radius"].GetDouble();
        m_width             = dp["Width"].GetDouble();
        m_lateral_stiffness = dp["Carcass Lateral Stiffness"].GetDouble();
        m_normalStiffness   = dp["Vertical Stiffness"].GetDouble();
        m_normalDamping     = dp["Vertical Damping"].GetDouble();
        m_rolling_resistance = dp["Rolling Resistance"].GetDouble();

        if (dp.HasMember("Vertical Curve Data")) {
            int num_points = dp["Vertical Curve Data"].Size();
            for (int i = 0; i < num_points; i++) {
                m_vert_map.AddPoint(dp["Vertical Curve Data"][i][0u].GetDouble(),
                                    dp["Vertical Curve Data"][i][1u].GetDouble());
            }
            m_has_vert_table = true;
        }
    }

    // -------------------------------------------------------------------------
    // STIREMOD Parameters  (Imperial units: lbs, inches, psi)
    // -------------------------------------------------------------------------
    if (d.HasMember("STIREMOD Parameters")) {
        const auto& sp = d["STIREMOD Parameters"];

        // Geometry & Load
        m_stiParams.Fzt = sp["Rated Load"].GetDouble();
        m_stiParams.Tw  = sp["Tread Width"].GetDouble();
        m_stiParams.Tp  = sp["Tire Pressure"].GetDouble();

        // Peak Friction vs Load (Eq 13)
        m_stiParams.B1x = sp["B1x"].GetDouble();
        m_stiParams.B3x = sp["B3x"].GetDouble();
        m_stiParams.B4x = sp["B4x"].GetDouble();
        m_stiParams.B1y = sp["B1y"].GetDouble();
        m_stiParams.B3y = sp["B3y"].GetDouble();
        m_stiParams.B4y = sp["B4y"].GetDouble();

        // Stiffness vs Load (Eq 4, 5)
        m_stiParams.A0    = sp["A0"].GetDouble();
        m_stiParams.A1    = sp["A1"].GetDouble();
        m_stiParams.A2    = sp["A2"].GetDouble();
        m_stiParams.Kx    = sp["Kx"].GetDouble();
        m_stiParams.CS_FZ = sp["CS_FZ"].GetDouble();

        // Camber Stiffness (Eq 8)
        m_stiParams.A3      = sp["A3"].GetDouble();
        m_stiParams.A4      = sp["A4"].GetDouble();
        m_stiParams.K_gamma = sp["K_gamma"].GetDouble();

        // Saturation Shape (Eq 6)
        m_stiParams.C1 = sp["C1"].GetDouble();
        m_stiParams.C2 = sp["C2"].GetDouble();
        m_stiParams.C3 = sp["C3"].GetDouble();
        m_stiParams.C4 = sp["C4"].GetDouble();
        m_stiParams.C5 = sp["C5"].GetDouble();

        // Friction Decay (Eq 12)
        m_stiParams.K_mux        = sp["K_mux"].GetDouble();
        m_stiParams.K_muy_offset = sp["K_muy_offset"].GetDouble();
        m_stiParams.K_muy_slope  = sp["K_muy_slope"].GetDouble();
        m_stiParams.K_muy_max_Fz = sp["K_muy_max_Fz"].GetDouble();

        // Misc
        m_stiParams.Ka = sp["Ka"].GetDouble();
        m_stiParams.K1 = sp["K1"].GetDouble();
        m_stiParams.G1 = sp["G1"].GetDouble();
        m_stiParams.G2 = sp["G2"].GetDouble();

        // Environmental (Skid Numbers)
        m_stiParams.SN_o = sp["SN_o"].GetDouble();
        m_stiParams.SN_t = sp["SN_t"].GetDouble();
    }

    // -------------------------------------------------------------------------
    // Bristle Parameters  (optional — SI units)
    // -------------------------------------------------------------------------
    if (d.HasMember("Bristle Parameters")) {
        const auto& bp = d["Bristle Parameters"];
        if (bp.HasMember("Sigma0"))
            m_sigma0 = bp["Sigma0"].GetDouble();
        if (bp.HasMember("Sigma1"))
            m_sigma1 = bp["Sigma1"].GetDouble();
    }

    // -------------------------------------------------------------------------
    // Visualization
    // -------------------------------------------------------------------------
    m_visualization_width = ChStiremodTire::GetVisualizationWidth();

    if (d.HasMember("Visualization")) {
        const auto& vis = d["Visualization"];

        if (vis.HasMember("Mesh Filename Left") && vis.HasMember("Mesh Filename Right")) {
            m_meshFile_left  = vis["Mesh Filename Left"].GetString();
            m_meshFile_right = vis["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }

        if (vis.HasMember("Width")) {
            m_visualization_width = vis["Width"].GetDouble();
        }
    }
}

// -----------------------------------------------------------------------------
void StiremodTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left, m_meshFile_right);
    } else {
        ChStiremodTire::AddVisualizationAssets(vis);
    }
}

void StiremodTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChStiremodTire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono