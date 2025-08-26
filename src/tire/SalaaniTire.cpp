#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "src/tire/SalaaniTire.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
SalaaniTire::SalaaniTire(const std::string& filename) : ChSalaaniTire(""), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

SalaaniTire::SalaaniTire(const rapidjson::Document& d) : ChSalaaniTire(""), m_has_mesh(false) {
    Create(d);
}

namespace {
void CheckHasMember(const rapidjson::Document& d, const std::string& member_name) {
  if (!d.HasMember(member_name.c_str())) {
    throw std::runtime_error("Missing required member '" + member_name + "' in JSON file");
  }
}
void CheckHasMember(const rapidjson::Value::ConstObject& d, const std::string& member_name) {
  if (!d.HasMember(member_name.c_str())) {
    throw std::runtime_error("Missing required member '" + member_name + "' in JSON file");
  }
}
}

void SalaaniTire::Create(const rapidjson::Document& d) {
  // Invoke base class method.
  ChPart::Create(d);

  // Read design parameters from top level (required)
  CheckHasMember(d, "Mass [kg]");
  m_mass = d["Mass [kg]"].GetDouble();
  CheckHasMember(d, "Inertia [kg.m2]");
  m_inertia = ReadVectorJSON(d["Inertia [kg.m2]"]);
  // This should be computed from tire aspect ratio, width, and rim radius.
  // CheckHasMember(d, "Unloaded Radius [m]");
  // m_unloaded_radius = d["Unloaded Radius [m]"].GetDouble();
  CheckHasMember(d, "Maximum Load [kg]");
  m_max_load = d["Maximum Load [kg]"].GetDouble();
  CheckHasMember(d, "Rim Radius [m]");
  m_rim_radius = d["Rim Radius [m]"].GetDouble();
  CheckHasMember(d, "Width [m]");
  m_width = d["Width [m]"].GetDouble();
  CheckHasMember(d, "Aspect Ratio [%]");
  m_aspect_ratio = d["Aspect Ratio [%]"].GetDouble();

  // Read tire parameters section (required)
  CheckHasMember(d, "SalaaniCoeffs");
  auto salaani_coeffs = d["SalaaniCoeffs"].GetObject();

  // Lateral stiffness parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_stiffness");
  auto lat_stiff = salaani_coeffs["lateral_stiffness"].GetObject();
  CheckHasMember(lat_stiff, "C1");
  m_salaani_coeff.C1 = lat_stiff["C1"].GetDouble();
  CheckHasMember(lat_stiff, "C2");
  m_salaani_coeff.C2 = lat_stiff["C2"].GetDouble();
  CheckHasMember(lat_stiff, "Cam");
  m_salaani_coeff.Cam = lat_stiff["Cam"].GetDouble();
  CheckHasMember(lat_stiff, "FZCam");
  m_salaani_coeff.FZCam = lat_stiff["FZCam"].GetDouble();

  // Longitudinal stiffness parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_stiffness");
  auto long_stiff = salaani_coeffs["longitudinal_stiffness"].GetObject();
  CheckHasMember(long_stiff, "Ckm");
  m_salaani_coeff.Ckm = long_stiff["Ckm"].GetDouble();
  CheckHasMember(long_stiff, "FZCKM");
  m_salaani_coeff.FZCKM = long_stiff["FZCKM"].GetDouble();
  CheckHasMember(long_stiff, "n_val");
  m_salaani_coeff.n_val = long_stiff["n_val"].GetDouble();

  // Lateral peak friction parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_peak_friction");
  auto lat_peak_friction = salaani_coeffs["lateral_peak_friction"].GetObject();
  CheckHasMember(lat_peak_friction, "eta1_lat");
  m_salaani_coeff.eta1_lat = lat_peak_friction["eta1_lat"].GetDouble();
  CheckHasMember(lat_peak_friction, "eta2_lat");
  m_salaani_coeff.eta2_lat = lat_peak_friction["eta2_lat"].GetDouble();
  CheckHasMember(lat_peak_friction, "mu_p0_lat");
  m_salaani_coeff.mu_p0_lat = lat_peak_friction["mu_p0_lat"].GetDouble();

  // Longitudinal peak friction parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_peak_friction");
  auto long_peak_friction = salaani_coeffs["longitudinal_peak_friction"].GetObject();
  CheckHasMember(long_peak_friction, "eta0_long");
  m_salaani_coeff.eta0_long = long_peak_friction["eta0_long"].GetDouble();
  CheckHasMember(long_peak_friction, "eta1_long");
  m_salaani_coeff.eta1_long = long_peak_friction["eta1_long"].GetDouble();
  CheckHasMember(long_peak_friction, "mu_p0_long");
  m_salaani_coeff.mu_p0_long = long_peak_friction["mu_p0_long"].GetDouble();
  CheckHasMember(long_peak_friction, "FZ0");
  m_salaani_coeff.FZ0 = long_peak_friction["FZ0"].GetDouble();

  // Lateral friction decay parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_friction_decay");
  auto lat_friction_decay = salaani_coeffs["lateral_friction_decay"].GetObject();
  CheckHasMember(lat_friction_decay, "d1_lat");
  m_salaani_coeff.d1_lat = lat_friction_decay["d1_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "d2_lat");
  m_salaani_coeff.d2_lat = lat_friction_decay["d2_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "d3_lat");
  m_salaani_coeff.d3_lat = lat_friction_decay["d3_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "epsilon_sy");
  m_salaani_coeff.epsilon_sy = lat_friction_decay["epsilon_sy"].GetDouble();

  // Longitudinal friction decay parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_friction_decay");
  auto long_friction_decay = salaani_coeffs["longitudinal_friction_decay"].GetObject();
  CheckHasMember(long_friction_decay, "d1_long");
  m_salaani_coeff.d1_long = long_friction_decay["d1_long"].GetDouble();
  CheckHasMember(long_friction_decay, "d2_long");
  m_salaani_coeff.d2_long = long_friction_decay["d2_long"].GetDouble();
  CheckHasMember(long_friction_decay, "d3_long");
  m_salaani_coeff.d3_long = long_friction_decay["d3_long"].GetDouble();
  CheckHasMember(long_friction_decay, "epsilon_xx");
  m_salaani_coeff.epsilon_xx = long_friction_decay["epsilon_xx"].GetDouble();

  // Aligning moment parameters (required)
  CheckHasMember(salaani_coeffs, "aligning_moment");
  auto aligning_moment = salaani_coeffs["aligning_moment"].GetObject();
  CheckHasMember(aligning_moment, "tz1");
  m_salaani_coeff.tz1 = aligning_moment["tz1"].GetDouble();
  CheckHasMember(aligning_moment, "tz2");
  m_salaani_coeff.tz2 = aligning_moment["tz2"].GetDouble();
  CheckHasMember(aligning_moment, "epsilon_x");
  m_salaani_coeff.epsilon_x = aligning_moment["epsilon_x"].GetDouble();
  CheckHasMember(aligning_moment, "m1");
  m_salaani_coeff.m1 = aligning_moment["m1"].GetDouble();
  CheckHasMember(aligning_moment, "m0");
  m_salaani_coeff.m0 = aligning_moment["m0"].GetDouble();

  // Overturning moment parameters (required)
  CheckHasMember(salaani_coeffs, "overturning_moment");
  auto overturning_moment = salaani_coeffs["overturning_moment"].GetObject();
  CheckHasMember(overturning_moment, "tx1");
  m_salaani_coeff.tx1 = overturning_moment["tx1"].GetDouble();
  CheckHasMember(overturning_moment, "tx2");
  m_salaani_coeff.tx2 = overturning_moment["tx2"].GetDouble();
  CheckHasMember(overturning_moment, "tx3");
  m_salaani_coeff.tx3 = overturning_moment["tx3"].GetDouble();

  // Inclination angle parameters (required)
  CheckHasMember(salaani_coeffs, "inclination_angle");
  auto inclination_angle = salaani_coeffs["inclination_angle"].GetObject();
  CheckHasMember(inclination_angle, "Cr1");
  m_salaani_coeff.Cr1 = inclination_angle["Cr1"].GetDouble();
  CheckHasMember(inclination_angle, "Cr2");
  m_salaani_coeff.Cr2 = inclination_angle["Cr2"].GetDouble();

  // Additional parameters (required)
  CheckHasMember(salaani_coeffs, "additional_parameters");
  auto additional = salaani_coeffs["additional_parameters"].GetObject();
  CheckHasMember(additional, "Rt");
  m_salaani_coeff.Rt = additional["Rt"].GetDouble();
  CheckHasMember(additional, "Cz_contact");
  m_salaani_coeff.Cz_contact = additional["Cz_contact"].GetDouble();
  CheckHasMember(additional, "beta");
  m_salaani_coeff.beta = additional["beta"].GetDouble();
  CheckHasMember(additional, "plysteer");
  m_salaani_coeff.plysteer = additional["plysteer"].GetDouble();

  // Dahl friction model parameters (optional with defaults)
  if (d.HasMember("DahlCoeffs")) {
      auto dahl_coeffs = d["DahlCoeffs"].GetObject();
      m_dahl_coeff.sigma0 = dahl_coeffs.HasMember("sigma0") ? dahl_coeffs["sigma0"].GetDouble() : 100000.0;
      m_dahl_coeff.sigma1 = dahl_coeffs.HasMember("sigma1") ? dahl_coeffs["sigma1"].GetDouble() : 5000.0;
  } else {
      m_dahl_coeff.sigma0 = 100000.0; // Default values as defined in header
      m_dahl_coeff.sigma1 = 5000.0;
  }

  // // Read required vertical stiffness parameters
  // if (!d.HasMember("Vertical Stiffness")) {
  //     throw std::runtime_error("SalaaniTire::Create - Missing required 'Vertical Stiffness' section.\n"
  //                               "Required parameters: d1, d2");
  // }
  // auto vert_stiff = d["Vertical Stiffness"].GetObject();
  // if (!vert_stiff.HasMember("d1") || !vert_stiff.HasMember("d2")) {
  //     throw std::runtime_error("SalaaniTire::Create - Missing required parameters in 'Vertical Stiffness' section.\n"
  //                               "Required: d1, d2");
  // }

  // Read low-speed transition parameters (optional)
  if (d.HasMember("SpeedBasedTireModelBlending")) {
    auto speed_blending = d["SpeedBasedTireModelBlending"].GetObject();
    m_frblend_begin = speed_blending.HasMember("Friction Blend Begin [m/s]") ?
        speed_blending["Friction Blend Begin [m/s]"].GetDouble() : 0.1;
    m_frblend_end = speed_blending.HasMember("Friction Blend End [m/s]") ?
        speed_blending["Friction Blend End [m/s]"].GetDouble() : 1.0;
  }

  // Read startup transition parameters (optional)
  if (d.HasMember("TimeBasedTireModelBlending")) {
    auto time_blending = d["TimeBasedTireModelBlending"].GetObject();
    m_use_startup_transition = d.HasMember("Use Startup Transition") ? 
        d["Use Startup Transition"].GetBool() : false;   
    m_begin_start_transition = d.HasMember("Begin Start Transition [s]") ?
        d["Begin Start Transition [s]"].GetDouble() : 0.0;
    m_end_start_transition = d.HasMember("End Start Transition [s]") ?
        d["End Start Transition [s]"].GetDouble() : 0.25;
  }

  if (!d.HasMember("Visualization")) {
    throw std::runtime_error("SalaaniTire::Create - Missing required 'Visualization' section.\n");
  }
  auto visualization = d["Visualization"].GetObject();
  if (!visualization.HasMember("Mesh Filename Left") ||
      !visualization.HasMember("Mesh Filename Right")) {
    throw std::runtime_error("SalaaniTire::Create - Missing required parameters in 'Visualization' section.\n"
                             "Required: 'Mesh Filename Left', 'Mesh Filename Right'\n");
  }
  m_meshFile_left = visualization["Mesh Filename Left"].GetString();
  m_meshFile_right = visualization["Mesh Filename Right"].GetString();
  m_has_mesh = true;

  // Initialize effective radius to unloaded radius
  m_states.R_eff = m_unloaded_radius;
}

// -----------------------------------------------------------------------------
void SalaaniTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
      SalaaniTire::AddVisualizationAssets(vis);
    }
}

void SalaaniTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChSalaaniTire::RemoveVisualizationAssets();
}

}  // end namespace vehicle
}  // end namespace chrono
