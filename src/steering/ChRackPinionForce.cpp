#include <vector>
#include <iostream>
#include <cmath>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/functions/ChFunctionConst.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

#include "src/steering/ChRackPinionForce.h"
namespace chrono::vehicle
{

  // -----------------------------------------------------------------------------
  ChRackPinionForce::ChRackPinionForce(const std::string &name) : ChSteering(name) {}

  ChRackPinionForce::~ChRackPinionForce()
  {
    auto sys = m_motor->GetSystem();
    if (sys)
    {
      // sys->Remove(m_prismatic);
      sys->Remove(m_motor);
    }
  }

  // -----------------------------------------------------------------------------
  void ChRackPinionForce::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChVector3d& location,
                                      const ChQuaternion<>& rotation)
  {
    m_parent = chassis;
    m_rel_xform = ChFrame<>(location, rotation);

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrameRefToAbs());

    // Create and initialize the steering link body
    ChVector3d link_pos = steering_to_abs.TransformPointLocalToParent(ChVector3d(0, GetSteeringLinkCOM(), 0));
    ChQuaternion<> link_rot = steering_to_abs.GetRot().GetNormalized() * chrono::QuatFromAngleX(CH_PI_2);
    // ChQuaternion<> link_rot = chassisBody->GetRot().GetNormalized();

    m_link = std::shared_ptr<ChBody>(chrono_types::make_shared<ChBody>());
    m_link->SetName(m_name + "_link");
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
    const double angle_rad = CH_PI_2;  // 90 degrees in radians
    ChQuaternion<> rotation_z = chrono::QuatFromAngleZ(angle_rad);
    ChQuaternion<> new_link_rot = link_rot * rotation_z;
    auto link_frame = ChFrame<>(link_pos, new_link_rot);

    m_motor = chrono_types::make_shared<ChLinkMotorLinearForce>();
    m_motor->SetName(m_name + "_motor");
    m_motor->Initialize(m_link, chassisBody, false, link_frame, link_frame);
    sys->AddLink(m_motor);

    // Create and initialize the spring damper. This is used to model friction
    // which stabilizes the simulation of the steering rack.
    m_springDamper = chrono_types::make_shared<ChLinkTSDA>();
    m_springDamper->Initialize(
        m_link,              // First connected body
        chassisBody,         // Second connected body
        false,               // Use absolute coordinates
        ChVector3d(0, 0, 0), // Connection point on first body
        ChVector3d(0, 0, 0)  // Connection point on second body
    );
    m_springDamper->SetName("TSDA");
    m_springDamper->SetSpringCoefficient(0.0);           // Spring stiffness (N/m)
    m_springDamper->SetDampingCoefficient(GetDamping());          // Damping coefficient (Ns/m)
    m_springDamper->SetRestLength(0.0);                  // Initial separation between connection points
    sys->Add(m_springDamper);
  }

  // -----------------------------------------------------------------------------
  void ChRackPinionForce::Synchronize(double time, const DriverInputs &driver_inputs)
  {
    // Interpret the steering input as a torque and scale it by radius to get
    // linear force on the rack.
    const double pinion_radius = GetPinionRadius();
    const double force = driver_inputs.m_steering * pinion_radius;
    const double angle = GetPinionAngle();
    const ChVector3d reactive_force = m_motor->GetMotorForce();
    const double bumper_spring_constant = 100000.0;

    if (auto fun = std::dynamic_pointer_cast<ChFunctionConst>(
            m_motor->GetForceFunction())) {
      const auto max_angle = GetMaxAngle();
      // Enforce maximum displacement.
      if (std::abs(angle) > max_angle) {
        fun->SetConstant(-bumper_spring_constant * (angle - max_angle));
      } else {
        fun->SetConstant(force);
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
    m_xform = m_parent->GetTransform() * m_rel_xform;
  }

  // -----------------------------------------------------------------------------
  void ChRackPinionForce::AddVisualizationAssets(VisualizationType vis)
  {
    if (vis == VisualizationType::NONE)
      return;

    double length = GetSteeringLinkLength();

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(GetSteeringLinkRadius(), length);
    m_link->AddVisualShape(cyl);
  }

  void ChRackPinionForce::RemoveVisualizationAssets()
  {
    ChPart::RemoveVisualizationAssets(m_link);
  }

  // -----------------------------------------------------------------------------
  void ChRackPinionForce::LogConstraintViolations()
  {

    // Actuator
    {
      ChVectorDynamic<> C = m_motor->GetConstraintViolation();
      std::cout << "Actuator            ";
      std::cout << "  " << C(0) << "  ";
    }
  }

  double ChRackPinionForce::GetPinionAngle() {
    return m_motor->GetMotorPos() / GetPinionRadius();
  }

  // -----------------------------------------------------------------------------
  std::shared_ptr<ChLinkTSDA::ForceFunctor> ChRackPinionForce::GetSpringDamperForceElement() const {
    return m_springDamper->GetForceFunctor();
  }

  // -----------------------------------------------------------------------------
  void ChRackPinionForce::ExportComponentList(rapidjson::Document &jsonDocument) const
  {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    // joints.push_back(m_prismatic);
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
    // joints.push_back(m_prismatic);
    joints.push_back(m_motor);
    database.WriteJoints(joints);
  }

} // end namespace chrono::vehicle
