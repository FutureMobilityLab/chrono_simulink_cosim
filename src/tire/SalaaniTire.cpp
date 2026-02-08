#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "src/tire/SalaaniTire.h"
#include "src/utils/utils.h"

using namespace rapidjson;

namespace {
  constexpr double kFtToM = 0.3048;
}

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
  try{
    Create(d);
  } catch (const std::exception& e) {
    std::cout << "Failed to create SalaaniTire: '" << e.what() << "'\n";
    throw(e);
  }
}


double SalaaniTire::GetNormalStiffnessForce(double depth) const {
  /*
  double F = depth * m_d1 + depth * depth * m_d2;  // tire force
  double free_depth = m_unloaded_radius - m_bottom_radius;
  if (depth - free_depth > 0) {
      // std::cout << "At bottom, applying bottom_stiffness: " << m_bottom_stiffness << "\n"; 
      F += (depth - free_depth) * m_bottom_stiffness;  // add bottom contact force
  }
  return F;
  */
  if (m_has_vert_table) {
    return m_vert_map.GetVal(depth);
  } else {
    return m_normalStiffness * depth;
  }
}

double SalaaniTire::GetNormalDampingForce(double depth, double velocity) const {
  // return m_vert_damping * velocity;
  return m_normalDamping * velocity;
}

void SalaaniTire::Create(const rapidjson::Document& d) {
  // Invoke base class method.
  ChPart::Create(d);

  m_has_vert_table = false;

  // Read design parameters from top level (required)
  CheckHasMember(d, "Mass [kg]");
  m_mass = d["Mass [kg]"].GetDouble();
  CheckHasMember(d, "Inertia [kg.m2]");
  m_inertia = ReadVectorJSON(d["Inertia [kg.m2]"]);
  // This should be computed from tire aspect ratio, width, and rim radius.
  CheckHasMember(d, "Unloaded Radius [m]");
  m_unloaded_radius = d["Unloaded Radius [m]"].GetDouble();
  // CheckHasMember(d, "Maximum Load [kg]");
  // m_max_load = d["Maximum Load [kg]"].GetDouble();
  // CheckHasMember(d, "Rim Radius [m]");
  // m_rim_radius = d["Rim Radius [m]"].GetDouble();
  CheckHasMember(d, "Width [m]");
  m_width = d["Width [m]"].GetDouble();
  m_visualization_width = m_width;  // default to physical width
  // CheckHasMember(d, "Aspect Ratio [%]");
  // m_aspect_ratio = d["Aspect Ratio [%]"].GetDouble();
  CheckHasMember(d, "Vertical Stiffness");
  m_normalStiffness = d["Vertical Stiffness"].GetDouble();
  CheckHasMember(d, "Vertical Damping");
  m_normalDamping = d["Vertical Damping"].GetDouble();
  CheckHasMember(d, "Enable Lateral Relaxation");
  m_enable_lateral_relaxation = d["Enable Lateral Relaxation"].GetBool();
  // TODO: Support vertical map for stiffness and damping like in Pac89Tire.cpp.

  // Read tire parameters section (required)
  CheckHasMember(d, "SalaaniCoeffs");
  auto salaani_coeffs = d["SalaaniCoeffs"].GetObject();
  tire::Params salaani_model_params;

  // Lateral stiffness parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_stiffness");
  auto lat_stiff = salaani_coeffs["lateral_stiffness"].GetObject();
  CheckHasMember(lat_stiff, "C1");
  salaani_model_params.C1 = lat_stiff["C1"].GetDouble();
  CheckHasMember(lat_stiff, "C2");
  salaani_model_params.C2 = lat_stiff["C2"].GetDouble();
  CheckHasMember(lat_stiff, "Cam");
  salaani_model_params.Cam = lat_stiff["Cam"].GetDouble();
  CheckHasMember(lat_stiff, "FZCam");
  salaani_model_params.FZCam = lat_stiff["FZCam"].GetDouble();

  // Longitudinal stiffness parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_stiffness");
  auto long_stiff = salaani_coeffs["longitudinal_stiffness"].GetObject();
  CheckHasMember(long_stiff, "Ckm");
  salaani_model_params.Ckm = long_stiff["Ckm"].GetDouble();
  CheckHasMember(long_stiff, "FZCKM");
  salaani_model_params.FZCKM = long_stiff["FZCKM"].GetDouble();
  CheckHasMember(long_stiff, "n_val");
  salaani_model_params.n_val = long_stiff["n_val"].GetDouble();

  // Lateral peak friction parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_peak_friction");
  auto lat_peak_friction = salaani_coeffs["lateral_peak_friction"].GetObject();
  CheckHasMember(lat_peak_friction, "eta1_lat");
  salaani_model_params.eta1_lat = lat_peak_friction["eta1_lat"].GetDouble();
  CheckHasMember(lat_peak_friction, "eta2_lat");
  salaani_model_params.eta2_lat = lat_peak_friction["eta2_lat"].GetDouble();
  CheckHasMember(lat_peak_friction, "mu_p0_lat");
  salaani_model_params.mu_p0_lat = lat_peak_friction["mu_p0_lat"].GetDouble();

  // Longitudinal peak friction parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_peak_friction");
  auto long_peak_friction = salaani_coeffs["longitudinal_peak_friction"].GetObject();
  CheckHasMember(long_peak_friction, "eta1_long");
  salaani_model_params.eta1_long = long_peak_friction["eta1_long"].GetDouble();
  CheckHasMember(long_peak_friction, "eta2_long");
  salaani_model_params.eta2_long = long_peak_friction["eta2_long"].GetDouble();
  CheckHasMember(long_peak_friction, "mu_p0_long");
  salaani_model_params.mu_p0_long = long_peak_friction["mu_p0_long"].GetDouble();
  CheckHasMember(long_peak_friction, "FZ0");
  salaani_model_params.FZ0 = long_peak_friction["FZ0"].GetDouble();

  // Lateral friction decay parameters (required)
  CheckHasMember(salaani_coeffs, "lateral_friction_decay");
  auto lat_friction_decay = salaani_coeffs["lateral_friction_decay"].GetObject();
  CheckHasMember(lat_friction_decay, "d1_lat");
  salaani_model_params.d1_lat = lat_friction_decay["d1_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "d2_lat");
  salaani_model_params.d2_lat = lat_friction_decay["d2_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "d3_lat");
  salaani_model_params.d3_lat = lat_friction_decay["d3_lat"].GetDouble();
  CheckHasMember(lat_friction_decay, "epsilon_sy");
  salaani_model_params.epsilon_sy = lat_friction_decay["epsilon_sy"].GetDouble();

  // Longitudinal friction decay parameters (required)
  CheckHasMember(salaani_coeffs, "longitudinal_friction_decay");
  auto long_friction_decay = salaani_coeffs["longitudinal_friction_decay"].GetObject();
  CheckHasMember(long_friction_decay, "d1_long");
  salaani_model_params.d1_long = long_friction_decay["d1_long"].GetDouble();
  CheckHasMember(long_friction_decay, "d2_long");
  salaani_model_params.d2_long = long_friction_decay["d2_long"].GetDouble();
  CheckHasMember(long_friction_decay, "d3_long");
  salaani_model_params.d3_long = long_friction_decay["d3_long"].GetDouble();
  CheckHasMember(long_friction_decay, "epsilon_xx");
  salaani_model_params.epsilon_xx = long_friction_decay["epsilon_xx"].GetDouble();

  // Aligning moment parameters (required)
  CheckHasMember(salaani_coeffs, "aligning_moment");
  auto aligning_moment = salaani_coeffs["aligning_moment"].GetObject();
  CheckHasMember(aligning_moment, "tz1");
  salaani_model_params.tz1 = aligning_moment["tz1"].GetDouble();
  CheckHasMember(aligning_moment, "tz2");
  salaani_model_params.tz2 = aligning_moment["tz2"].GetDouble();
  CheckHasMember(aligning_moment, "epsilon_x");
  salaani_model_params.epsilon_x = aligning_moment["epsilon_x"].GetDouble();
  CheckHasMember(aligning_moment, "m1");
  salaani_model_params.m1 = aligning_moment["m1"].GetDouble();
  CheckHasMember(aligning_moment, "m0");
  salaani_model_params.m0 = aligning_moment["m0"].GetDouble();

  // Overturning moment parameters (required)
  CheckHasMember(salaani_coeffs, "overturning_moment");
  auto overturning_moment = salaani_coeffs["overturning_moment"].GetObject();
  CheckHasMember(overturning_moment, "tx1");
  salaani_model_params.tx1 = overturning_moment["tx1"].GetDouble();
  CheckHasMember(overturning_moment, "tx2");
  salaani_model_params.tx2 = overturning_moment["tx2"].GetDouble();
  CheckHasMember(overturning_moment, "tx3");
  salaani_model_params.tx3 = overturning_moment["tx3"].GetDouble();

  // Inclination angle parameters (required)
  CheckHasMember(salaani_coeffs, "inclination_angle");
  auto inclination_angle = salaani_coeffs["inclination_angle"].GetObject();
  CheckHasMember(inclination_angle, "Cr1");
  salaani_model_params.Cr1 = inclination_angle["Cr1"].GetDouble();
  CheckHasMember(inclination_angle, "Cr2");
  salaani_model_params.Cr2 = inclination_angle["Cr2"].GetDouble();

  // Additional parameters (required)
  CheckHasMember(salaani_coeffs, "additional_parameters");
  auto additional = salaani_coeffs["additional_parameters"].GetObject();
  CheckHasMember(additional, "Rt");
  salaani_model_params.Rt = additional["Rt"].GetDouble();
  CheckHasMember(additional, "Cz_contact");
  salaani_model_params.Cz_contact = additional["Cz_contact"].GetDouble();
  CheckHasMember(additional, "beta");
  salaani_model_params.beta = additional["beta"].GetDouble();
  // TODO: It would be better to use beta directly in ChSalaaniTire.
  m_lateral_relaxation_length = additional["beta"].GetDouble() * kFtToM;
  CheckHasMember(additional, "plysteer");
  salaani_model_params.plysteer = additional["plysteer"].GetDouble();

  // Friction coefficients (required)
  CheckHasMember(salaani_coeffs, "validation_range");
  auto validation_range = salaani_coeffs["validation_range"].GetObject();
  CheckHasMember(validation_range, "MUNTEST");
  salaani_model_params.MUNTEST = validation_range["MUNTEST"].GetDouble();
  CheckHasMember(validation_range, "FzMax");
  salaani_model_params.FzMax = validation_range["FzMax"].GetDouble();

  // Initialize the salaani model
  std::cout << "initializing tire.\n";
  try {
    salaani_model = std::make_unique<tire::SalaaniTireModel>(salaani_model_params);
  } catch (const std::exception& e) {
    std::cout << " Got invalid parameters from JSON: " << e.what() << "\n";
  }

  // Dahl friction model parameters (optional with defaults)
  if (d.HasMember("DahlCoeffs")) {
      auto dahl_coeffs = d["DahlCoeffs"].GetObject();
      m_dahl_coeff.sigma0 = dahl_coeffs.HasMember("sigma0") ? dahl_coeffs["sigma0"].GetDouble() : 100000.0;
      m_dahl_coeff.sigma1 = dahl_coeffs.HasMember("sigma1") ? dahl_coeffs["sigma1"].GetDouble() : 5000.0;
  } else {
      m_dahl_coeff.sigma0 = 100000.0; // Default values as defined in header
      m_dahl_coeff.sigma1 = 5000.0;
  }

  // Read low-speed transition parameters (optional)
  if (d.HasMember("SpeedBasedTireModelBlending")) {
    auto speed_blending = d["SpeedBasedTireModelBlending"].GetObject();
    CheckHasMember(speed_blending, "Enable");
    m_enable_speed_based_tire_model_blending = speed_blending["Enable"].GetBool();
    CheckHasMember(speed_blending, "Friction Blend Begin [m/s]");
    m_frblend_begin = speed_blending["Friction Blend Begin [m/s]"].GetDouble();
    CheckHasMember(speed_blending, "Friction Blend End [m/s]");
    m_frblend_end = speed_blending["Friction Blend End [m/s]"].GetDouble();
  }

  m_has_mesh = false;
  m_has_vis_override = false;
  m_vis_override = VisualizationType::MESH;

  if (d.HasMember("Visualization")) {
    auto visualization = d["Visualization"].GetObject();

    // Optional: explicit visualization type override inside the tire JSON.
    if (visualization.HasMember("Visualization Type")) {
      auto vis_str = std::string(visualization["Visualization Type"].GetString());
      if (vis_str == "PRIMITIVES" || vis_str == "Primitives" || vis_str == "primitives") {
        m_vis_override = VisualizationType::PRIMITIVES;
        m_has_vis_override = true;
      } else if (vis_str == "NONE" || vis_str == "None" || vis_str == "none") {
        m_vis_override = VisualizationType::NONE;
        m_has_vis_override = true;
      } else {
        m_vis_override = VisualizationType::MESH;
        m_has_vis_override = true;
      }
    }

    if (visualization.HasMember("Width")) {
      m_visualization_width = visualization["Width"].GetDouble();
    }

    // Mesh filenames are optional; only enable mesh visualization if both are present
    if (visualization.HasMember("Mesh Filename Left") && visualization.HasMember("Mesh Filename Right")) {
      m_meshFile_left = visualization["Mesh Filename Left"].GetString();
      m_meshFile_right = visualization["Mesh Filename Right"].GetString();
      m_has_mesh = true;
    }
  } else {
    std::cout << "SalaaniTire::Create - No 'Visualization' section found. Defaulting to primitive visualization.\n";
  }

  // Initialize effective radius to unloaded radius
  m_states.R_eff = m_unloaded_radius;
}

// -----------------------------------------------------------------------------
void SalaaniTire::AddVisualizationAssets(VisualizationType vis) {
    VisualizationType resolved_vis = vis;
    if (m_has_vis_override) {
        resolved_vis = m_vis_override;
    }

    if (resolved_vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChSalaaniTire::AddVisualizationAssets(resolved_vis);
    }
}

void SalaaniTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChSalaaniTire::RemoveVisualizationAssets();
}

}  // end namespace vehicle
}  // end namespace chrono
