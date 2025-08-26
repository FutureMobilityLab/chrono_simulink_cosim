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

    virtual void SetSalaaniParams() override {}
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_mass;
    ChVector3d m_inertia;

    double m_visualization_width;
    bool m_has_mesh;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
