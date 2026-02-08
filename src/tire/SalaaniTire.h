#ifndef SALAANI_TIRE_H
#define SALAANI_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "src/tire/ChSalaaniTire.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// TMeasy tire constructed with data from file (JSON format).
class CH_VEHICLE_API SalaaniTire : public ChSalaaniTire {
  public:
    SalaaniTire(const std::string& filename);
    SalaaniTire(const rapidjson::Document& d);
    ~SalaaniTire() {}

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const override final;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const override final;

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }
    
    virtual void SetSalaaniParams() override { m_measured_side = LEFT; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_normalStiffness;
    double m_normalDamping;
    double m_mass;
    ChVector3d m_inertia;
    bool m_has_mesh;
    bool m_has_vis_override;
    VisualizationType m_vis_override;
    bool m_has_vert_table;
    ChFunctionInterp m_vert_map;

    double m_visualization_width;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
