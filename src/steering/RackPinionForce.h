// RackPinionForce object from JSON.

#ifndef RACK_PINIONFORCE_H
#define RACK_PINIONFORCE_H

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/core/ChVector3.h"

#include "src/steering/ChRackPinionForce.h"

namespace chrono
{
  namespace vehicle
  {
    /// Rack-pinion steering model constructed with data from file (JSON format).
    class CH_VEHICLE_API RackPinionForce : public ChRackPinionForce
    {
    public:
      RackPinionForce(const std::string &filename);
      RackPinionForce(const rapidjson::Document &d);
      ~RackPinionForce() {}

    protected:
      virtual double GetDamping() const override { return m_damping; }
      virtual double GetSpringCoefficient() const override { return m_springDamper->GetSpringCoefficient(); }
      virtual double GetSteeringLinkMass() const override { return m_steeringLinkMass; }
      virtual ChVector3<double> GetSteeringLinkInertia() const override { return m_steeringLinkInertia; }
      virtual double GetSteeringLinkCOM() const override { return m_steeringLinkCOM; }
      virtual double GetSteeringLinkRadius() const override { return m_steeringLinkRadius; }
      virtual double GetSteeringLinkLength() const override { return m_steeringLinkLength; }

      virtual double GetPinionRadius() const override { return m_pinionRadius; }

      virtual double GetMaxAngle() const override { return m_maxAngle; }

    private:
      virtual void Create(const rapidjson::Document &d) override;

      double m_steeringLinkMass;
      ChVector3<double> m_steeringLinkInertia;
      double m_steeringLinkCOM;
      double m_steeringLinkRadius;
      double m_steeringLinkLength;
      double m_damping;

      double m_pinionRadius;
      double m_maxAngle;
    };

  } // end namespace vehicle
} // end namespace chrono

#endif
