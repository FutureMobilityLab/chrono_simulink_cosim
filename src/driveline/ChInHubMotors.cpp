#include "ChInHubMotors.h"

namespace chrono::vehicle
{

  void ChInHubMotors::Initialize(std::shared_ptr<ChChassis> chassis,
                                 const ChAxleList &axles)
  {
    if (axles.size() < 4)
    {
      throw std::runtime_error("ChInHubMotors requires at least 4 axles for initialization.");
    }

    this->chassis = chassis;

    motors.clear();
    for (size_t i = 0; i < 4; ++i)
    {
      auto spindle = axles[i]->GetWheel(LEFT)->GetSpindle();
      auto motor = std::make_shared<chrono::ChLinkMotorRotationTorque>();
      motor->Initialize(chassis->GetBody(), spindle, chrono::ChFrame<>(spindle->GetPos()));
      motors.push_back(motor);
    }
  }

  void ChInHubMotors::Synchronize(double time, const std::vector<double> &torques)
  {
    if (torques.size() != motors.size())
    {
      throw std::runtime_error("Mismatch between torque vector size and number of motors.");
    }

    for (size_t i = 0; i < torques.size(); ++i)
    {
      applyTorque(static_cast<WheelIndex>(i), torques[i]);
    }
  }

  void ChInHubMotors::applyTorque(WheelIndex wheel_index, double torque)
  {
    if (wheel_index < 0 || wheel_index >= motors.size())
    {
      throw std::runtime_error("Invalid wheel index.");
    }

    auto motor = motors[wheel_index];
    motor->SetTorqueFunction(std::make_shared<chrono::ChFunction_Const>(torque));
  }

  std::shared_ptr<chrono::ChLinkMotorRotationTorque> ChInHubMotors::GetMotor(WheelIndex wheel_index) const
  {
    if (wheel_index < 0 || wheel_index >= motors.size())
    {
      throw std::runtime_error("Invalid wheel index.");
    }
    return motors[wheel_index];
  }

  double ChInHubMotors::GetMotorTorque(WheelIndex wheel_index) const
  {
    if (wheel_index < 0 || wheel_index >= motors.size())
    {
      throw std::runtime_error("Invalid wheel index.");
    }
    return motors[wheel_index]->GetMotorTorque();
  }

} // namespace chrono::vehicle
