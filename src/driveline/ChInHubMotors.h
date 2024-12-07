#ifndef CHINHUBMOTORS_H
#define CHINHUBMOTORS_H

#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include <stdexcept>
#include <memory>
#include <vector>

namespace chrono
{
  namespace vehicle
  {

    /**
     * @class ChInHubMotors
     * @brief Base class for vehicles with in-wheel hub motors.
     *
     * This class provides a generic interface for in-wheel hub motor systems.
     */
    class ChInHubMotors : public ChPart
    {
    public:
      /**
       * @enum WheelIndex
       * @brief Enumeration for indexing the wheels.
       */
      enum WheelIndex
      {
        FRONT_LEFT = 0,  ///< Front-left wheel
        FRONT_RIGHT = 1, ///< Front-right wheel
        REAR_LEFT = 2,   ///< Rear-left wheel
        REAR_RIGHT = 3   ///< Rear-right wheel
      };

      /**
       * @brief Default constructor.
       */
      ChInHubMotors() = default;

      /**
       * @brief Virtual destructor.
       */
      virtual ~ChInHubMotors() = default;

      /**
       * @brief Initialize the in-hub motor system.
       *
       * @param chassis A shared pointer to the associated chassis subsystem.
       * @param axles The list of all vehicle axle subsystems.
       * @throw std::runtime_error if the number of axles is less than 4.
       */
      virtual void Initialize(std::shared_ptr<ChChassis> chassis,
                              const ChAxleList &axles);

      /**
       * @brief Synchronize the motors with the simulation by applying external torques.
       *
       * @param time The current simulation time (unused).
       * @param torques A vector of torques to apply to the motors.
       * @throw std::runtime_error if the size of torques does not match the number of motors.
       */
      virtual void Synchronize(double time, const std::vector<double> &torques);

      /**
       * @brief Apply torque to a specific wheel motor.
       *
       * @param wheel_index The index of the wheel motor.
       * @param torque The torque value to apply.
       * @throw std::runtime_error if the wheel index is invalid.
       */
      virtual void applyTorque(WheelIndex wheel_index, double torque);

      /**
       * @brief Get a pointer to a specific motor.
       *
       * @param wheel_index The index of the wheel motor (0-3).
       * @return A shared pointer to the motor.
       * @throw std::runtime_error if the wheel index is invalid.
       */
      std::shared_ptr<chrono::ChLinkMotorRotationTorque> GetMotor(WheelIndex wheel_index) const;

      /**
       * @brief Get the torque applied to a specific motor.
       *
       * @param wheel_index The index of the wheel motor (0-3).
       * @return The applied torque value.
       * @throw std::runtime_error if the wheel index is invalid.
       */
      double GetMotorTorque(WheelIndex wheel_index) const;

    protected:
      std::shared_ptr<ChChassis> chassis;                                     ///< Pointer to the associated chassis.
      std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> motors; ///< Vector of motor torque links.
    };

  } // namespace vehicle
} // namespace chrono

#endif // CHINHUBMOTORS_H
