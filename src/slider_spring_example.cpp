#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChLinkMotorLinearForce.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <iostream>

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(RackPinionSimulation)

class RackPinionSimulation {
  private:
    // Chrono simulation system
    ChSystemNSC m_system;

    // Bodies
    std::shared_ptr<ChBodyEasyBox> m_rack;
    std::shared_ptr<ChBodyEasyBox> m_ground;

    // Constraints and connections
    std::shared_ptr<ChLinkTSDA> m_springDamper;
    std::shared_ptr<ChLinkMotorLinearForce> m_forceActuator;
    // std::shared_ptr<ChLinkLockPrismatic> m_rackConstraint;

    chrono::ChRealtimeStepTimer m_realtime_timer;
    std::shared_ptr<irrlicht::ChVisualSystemIrrlicht> m_vis;

  public:
    RackPinionSimulation() {
      // System setup
      m_system.Set_G_acc(ChVector<>(0, 0, 0));
      
      // Create bodies
      CreateBodies();
      CreateConstraints();

      // Create the Irrlicht visualization system
      m_vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
      m_vis->AttachSystem(&m_system);
      m_vis->SetWindowSize(800, 600);
      m_vis->SetWindowTitle("Motors");
      m_vis->Initialize();
      m_vis->AddLogo();
      m_vis->AddSkyBox();
      m_vis->AddCamera(chrono::ChVector<>(1, 3, -7));
      m_vis->AddTypicalLights();
      m_vis->AddLightWithShadow(chrono::ChVector<>(20.0, 35.0, -25.0), chrono::ChVector<>(0, 0, 0), 55, 20, 55, 35, 512,
                              chrono::ChColor(0.6f, 0.8f, 1.0f));
      m_vis->EnableShadows();
    }

    ChSystemNSC* GetSystem() {
      return &m_system;
    }

    void CreateBodies() {
      auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 2, 20, 3000);
      floorBody->SetPos(ChVector<>(0, -2, 0));
      floorBody->SetBodyFixed(true);
      floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
      m_system.Add(floorBody);

      // Ground body
      m_ground = chrono_types::make_shared<ChBodyEasyBox>(4., 0.3, 0.6, 1000);
      m_ground->SetNameString("ground");
      m_ground->SetBodyFixed(true);
      m_system.Add(m_ground);

      // Rack body
      m_rack = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.2, 0.5, 1000);
      m_rack->SetNameString("rack");
      // m_rack->SetMass(2.0);
      // m_rack->SetInertiaXX(ChVector<>(0.5, 0.5, 0.5));
      m_rack->SetPos(ChVector<>(0, 0.3, 0));
      m_rack->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
      m_system.Add(m_rack);
    }

    void CreateConstraints() {
      // // Prismatic constraint for rack (only allow X-axis translation)
      // m_rackConstraint = chrono_types::make_shared<chrono::ChLinkLockPrismatic>();
      // m_rackConstraint->SetNameString("RackConstraint");
      // m_rackConstraint->Initialize(m_rack, m_ground, 
      //     chrono::ChCoordsys<>(chrono::ChVector<>(0, 0, 0), 
      //     chrono::Q_from_AngAxis(0, chrono::VECT_Z)));
      // m_system.Add(m_rackConstraint);

      // TSDA Spring-Damper Constraint for Rack
      m_springDamper = chrono_types::make_shared<ChLinkTSDA>();
      m_springDamper->Initialize(
          m_rack,     // First connected body
          m_ground,   // Second connected body
          false,      // Use absolute coordinates
          ChVector<>(0, 0, 0),   // Connection point on first body
          ChVector<>(0, 0, 0)    // Connection point on second body
      );
      m_springDamper->SetNameString("TSDA");
      m_springDamper->SetSpringCoefficient(5000.0);  // Spring stiffness (N/m)
      m_springDamper->SetDampingCoefficient(100.0);  // Damping coefficient (Ns/m)
      m_springDamper->SetRestLength(0.0);  // Initial separation between connection points
      m_system.Add(m_springDamper);

      // Linear Force Actuator
      m_forceActuator = chrono_types::make_shared<ChLinkMotorLinearForce>();
      m_forceActuator->Initialize(m_rack, m_ground, ChFrame<>(0., 0., 0.));
      m_forceActuator->SetNameString("LinearForceActuator");
      m_system.Add(m_forceActuator);
      
      // Create a constant torque function
      auto forceFun = chrono_types::make_shared<ChFunction_Const>(5000.0);
      m_forceActuator->SetForceFunction(forceFun);
    }

    void Simulate(double totalTime, double timeStep) {
      // Simulation loop
      GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
      m_system.ShowHierarchy(GetLog());

      for (double t = 0; t < totalTime; t += timeStep) {
        if (!m_vis->Run()) {
          break;
        }

        // Render
        m_vis->BeginScene();
        m_vis->Render();
        m_vis->EndScene();

        // Advance simulation
        m_system.DoStepDynamics(timeStep);
        
        if (std::fmod(t, 0.1) < timeStep){
          std::cout << "Time:" << t 
                    << "\tInput Force:" << m_forceActuator->GetMotorForce()
                    << "\tRack Displacement: " << m_rack->GetPos() 
                    << "\tRack Velocity: " << m_rack->GetPos_dt()
                    << "\tSpring Force: " << m_springDamper->GetForce() 
                    << std::endl;
        }
        m_realtime_timer.Spin(timeStep);
      }
    }

    // Getter for final rack displacement
    double GetRackDisplacement() {
      return m_rack->GetPos().x();
    }
};

} // end namespace chrono

int main() {
  chrono::RackPinionSimulation simulation;

  // Simulate for 5 seconds with 1ms time step
  simulation.Simulate(10.0, 0.01);

  std::cout << "Final Rack Displacement: " 
            << simulation.GetRackDisplacement() << " meters" << std::endl;

  return 0;
}