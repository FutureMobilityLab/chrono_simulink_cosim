// =============================================================================
// Authors: Trevor Vidano
// =============================================================================
//
// STIREMOD tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef STIREMOD_TIRE_H
#define STIREMOD_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_thirdparty/rapidjson/document.h"

#include "src/tire/ChStiremodTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// STIREMOD tire model from JSON file.
class CH_VEHICLE_API StiremodTire : public ChStiremodTire {
  public:
    StiremodTire(const std::string& filename);
    StiremodTire(const rapidjson::Document& d);
    ~StiremodTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override {
        if (m_has_vert_table) {
            return m_vert_map.GetVal(depth);
        } else {
            return m_normalStiffness * depth;
        }
    }
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }

    virtual void SetStiremodParams() override { m_measured_side = LEFT; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_normalStiffness;
    double m_normalDamping;
    double m_mass;
    ChVector3d m_inertia;
    bool m_has_mesh;
    bool m_has_vert_table;
    ChFunctionInterp m_vert_map;

    double m_visualization_width;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // namespace vehicle
}  // end namespace chrono

#endif