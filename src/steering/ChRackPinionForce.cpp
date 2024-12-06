#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "src/steering/ChRackPinionForce.h"
namespace chrono
{
  namespace vehicle
  {

    // -----------------------------------------------------------------------------
    ChRackPinionForce::ChRackPinionForce(const std::string &name) : ChSteering(name) {}

    ChRackPinionForce::~ChRackPinionForce()
    {
      auto sys = m_prismatic->GetSystem();
      if (sys)
      {
        sys->Remove(m_prismatic);
        sys->Remove(m_motor);
      }
    }

    // -----------------------------------------------------------------------------
    void ChRackPinionForce::Initialize(std::shared_ptr<ChChassis> chassis,
                                       const ChVector<> &location,
                                       const ChQuaternion<> &rotation)
    {
      m_parent = chassis;
      m_rel_xform = ChFrame<>(location, rotation);

      auto chassisBody = chassis->GetBody();
      auto sys = chassisBody->GetSystem();

      // Express the steering reference frame in the absolute coordinate system.
      ChFrame<> steering_to_abs(location, rotation);
      steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrame_REF_to_abs());

      // Create and initialize the steering link body
      ChVector<> link_pos = steering_to_abs.TransformPointLocalToParent(ChVector<>(0, GetSteeringLinkCOM(), 0));
      ChQuaternion<> link_rot = steering_to_abs.GetRot().GetNormalized();

      m_link = std::shared_ptr<ChBody>(sys->NewBody());
      m_link->SetNameString(m_name + "_link");
      m_link->SetPos(link_pos);
      m_link->SetRot(link_rot);
      m_link->SetMass(GetSteeringLinkMass());
      m_link->SetInertiaXX(GetSteeringLinkInertia());
      sys->AddBody(m_link);

      // Create and initialize the linear actuator.
      // Y-axis for the chassis and rack point along the direction of the slide,
      // we need to rotate these frames by 90 degrees about the z axis so that
      // the x axis point along the direction (which is the convention for
      // ChLinkMotorLinearForce).
      ChQuaternion<> new_link_rot = link_rot * Q_from_AngZ(CH_C_PI_2);
      auto link_frame = ChFrame<>(link_pos, new_link_rot);

      m_motor = chrono_types::make_shared<ChLinkMotorLinearForce>();
      m_motor->SetNameString(m_name + "_motor");
      m_motor->Initialize(m_link, chassisBody, false, link_frame, link_frame);
      sys->AddLink(m_motor);

      // Create and initialize the spring damper. This is used to model friction
      // which stabilizes the simulation of the steering rack.
      m_springDamper = chrono_types::make_shared<ChLinkTSDA>();
      m_springDamper->Initialize(
          m_link,              // First connected body
          chassisBody,         // Second connected body
          false,               // Use absolute coordinates
          ChVector<>(0, 0, 0), // Connection point on first body
          ChVector<>(0, 0, 0)  // Connection point on second body
      );
      m_springDamper->SetNameString("TSDA");
      m_springDamper->SetSpringCoefficient(0.0);           // Spring stiffness (N/m)
      m_springDamper->SetDampingCoefficient(GetDamping()); // Damping coefficient (Ns/m)
      m_springDamper->SetRestLength(0.0);                  // Initial separation between connection points
      sys->Add(m_springDamper);
    }

    // -----------------------------------------------------------------------------
    void ChRackPinionForce::Synchronize(double time, const DriverInputs &driver_inputs)
    {
      // Interpret the steering input as a torque and scale it by radius to get
      // linear force on the rack.
      double force = driver_inputs.m_steering * GetPinionRadius() * 2000.;
      double angle = m_motor->GetMotorPos() / GetPinionRadius();

      if (auto fun = std::dynamic_pointer_cast<ChFunction_Const>(
              m_motor->GetForceFunction()))
      {
        // Enforce maximum displacement.
        if (angle > GetMaxAngle())
        {
          fun->Set_yconst(-force);
        }
        else if (angle < -GetMaxAngle())
        {
          fun->Set_yconst(-force);
        }
        else
        {
          fun->Set_yconst(force);
        }
      }
    }

    void ChRackPinionForce::InitializeInertiaProperties()
    {
      m_mass = GetSteeringLinkMass();
      m_com = ChFrame<>(GetSteeringLinkCOM(), QUNIT);
      m_inertia.setZero();
      m_inertia.diagonal() = GetSteeringLinkInertia().eigen();
    }

    void ChRackPinionForce::UpdateInertiaProperties()
    {
      m_parent->GetTransform().TransformLocalToParent(m_rel_xform, m_xform);
    }

    // -----------------------------------------------------------------------------
    void ChRackPinionForce::AddVisualizationAssets(VisualizationType vis)
    {
      if (vis == VisualizationType::NONE)
        return;

      double length = GetSteeringLinkLength();

      auto cyl = chrono_types::make_shared<ChCylinderShape>();
      cyl->GetCylinderGeometry().p1 = ChVector<>(0, length / 2, 0);
      cyl->GetCylinderGeometry().p2 = ChVector<>(0, -length / 2, 0);
      cyl->GetCylinderGeometry().rad = GetSteeringLinkRadius();
      m_link->AddVisualShape(cyl);
    }

    void ChRackPinionForce::RemoveVisualizationAssets()
    {
      ChPart::RemoveVisualizationAssets(m_link);
    }

    // -----------------------------------------------------------------------------
    void ChRackPinionForce::LogConstraintViolations()
    {
      // Translational joint
      {
        ChVectorDynamic<> C = m_prismatic->GetConstraintViolation();
        GetLog() << "Prismatic           ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
      }

      // Actuator
      {
        ChVectorDynamic<> C = m_motor->GetConstraintViolation();
        GetLog() << "Actuator            ";
        GetLog() << "  " << C(0) << "  ";
      }
    }

    // -----------------------------------------------------------------------------
    void ChRackPinionForce::ExportComponentList(rapidjson::Document &jsonDocument) const
    {
      ChPart::ExportComponentList(jsonDocument);

      std::vector<std::shared_ptr<ChBody>> bodies;
      bodies.push_back(m_link);
      ChPart::ExportBodyList(jsonDocument, bodies);

      std::vector<std::shared_ptr<ChLink>> joints;
      joints.push_back(m_prismatic);
      joints.push_back(m_motor);
      ChPart::ExportJointList(jsonDocument, joints);
    }

    void ChRackPinionForce::Output(ChVehicleOutput &database) const
    {
      if (!m_output)
        return;

      std::vector<std::shared_ptr<ChBody>> bodies;
      bodies.push_back(m_link);
      database.WriteBodies(bodies);

      std::vector<std::shared_ptr<ChLink>> joints;
      joints.push_back(m_prismatic);
      joints.push_back(m_motor);
      database.WriteJoints(joints);
    }

  } // end namespace vehicle
} // end namespace chrono
