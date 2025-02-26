#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "src/vehicle/WheeledVehicleForce.h"

namespace py = pybind11;

namespace chrono {
    class VehicleInterface {
    public:
        VehicleInterface(const std::string& vehicle_json, const std::string& engine_json, const std::string& transmission_json, 
                         const std::vector<std::string>& tire_jsons, ChContactMethod contact_method)
            : m_vehicle(vehicle_json, contact_method) {
            
            m_vehicle.Initialize(ChCoordsys<>(ChVector3<>(0, 0, 0.5), chrono::QuatFromAngleZ(0)));
            m_vehicle.GetChassis()->SetFixed(false);

            // Load powertrain
            auto engine = chrono::vehicle::ReadEngineJSON(chrono::vehicle::GetDataFile(engine_json));
            auto transmission = chrono::vehicle::ReadTransmissionJSON(chrono::vehicle::GetDataFile(transmission_json));
            m_powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
            m_vehicle.InitializePowertrain(m_powertrain);
            
            // Load tires
            for (unsigned int i = 0; i < m_vehicle.GetNumberAxles(); i++) {
                for (auto &wheel : m_vehicle.GetAxle(i)->GetWheels()) {
                    auto tire = chrono::vehicle::ReadTireJSON(chrono::vehicle::GetDataFile(tire_jsons[i]));
                    m_vehicle.InitializeTire(tire, wheel, chrono::vehicle::VisualizationType::MESH);
                }
            }
        }

        void SetInputs(double steering, double throttle, double braking) {
            m_inputs.m_steering = steering;
            m_inputs.m_throttle = throttle;
            m_inputs.m_braking = braking;
        }

        void Advance(double step_size) {
            m_vehicle.Synchronize(m_vehicle.GetSystem()->GetChTime(), m_inputs, m_terrain);
            m_vehicle.Advance(step_size);
        }

        std::vector<double> GetState() const {
            std::vector<double> data_out(39);
            auto chassis = m_vehicle.GetChassisBody();
            ChCoordsys chassis_frame = chassis->GetCoordsys();
            ChVector3<double> pos_dt = chassis->GetPosDt();
            ChVector3<double> pos_dtdt = chassis->GetLinAcc();
            
            data_out[0] = chassis->GetPos().x();
            data_out[1] = chassis->GetPos().y();
            data_out[2] = chassis->GetPos().z();
            data_out[3] = chassis->GetRot().GetCardanAnglesXYZ().x();
            data_out[4] = chassis->GetRot().GetCardanAnglesXYZ().y();
            data_out[5] = chassis->GetRot().GetCardanAnglesXYZ().z();
            data_out[6] = chassis_frame.TransformDirectionParentToLocal(pos_dt).x();
            data_out[7] = chassis_frame.TransformDirectionParentToLocal(pos_dt).y();
            data_out[8] = chassis_frame.TransformDirectionParentToLocal(pos_dt).z();
            data_out[9] = chassis->GetAngVelLocal().x();
            data_out[10] = chassis->GetAngVelLocal().y();
            data_out[11] = chassis->GetAngVelLocal().z();
            data_out[12] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).x();
            data_out[13] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).y();
            data_out[14] = chassis_frame.TransformDirectionParentToLocal(pos_dtdt).z();
            data_out[15] = chassis->GetAngAccLocal().x();
            data_out[16] = chassis->GetAngAccLocal().y();
            data_out[17] = chassis->GetAngAccLocal().z();
            data_out[18] = m_vehicle.GetSpindleOmega(0, chrono::vehicle::LEFT);
            data_out[19] = m_vehicle.GetSpindleOmega(0, chrono::vehicle::RIGHT);
            data_out[20] = m_vehicle.GetSpindleOmega(1, chrono::vehicle::LEFT);
            data_out[21] = m_vehicle.GetSpindleOmega(1, chrono::vehicle::RIGHT);
            data_out[22] = m_vehicle.GetAxle(0)->GetWheel(chrono::vehicle::LEFT)->GetTire()->GetLongitudinalSlip();
            data_out[23] = m_vehicle.GetAxle(0)->GetWheel(chrono::vehicle::RIGHT)->GetTire()->GetLongitudinalSlip();
            data_out[24] = m_vehicle.GetAxle(1)->GetWheel(chrono::vehicle::LEFT)->GetTire()->GetLongitudinalSlip();
            data_out[25] = m_vehicle.GetAxle(1)->GetWheel(chrono::vehicle::RIGHT)->GetTire()->GetLongitudinalSlip();
            data_out[26] = m_vehicle.GetAxle(0)->GetWheel(chrono::vehicle::LEFT)->GetTire()->GetSlipAngle();
            data_out[27] = m_vehicle.GetAxle(0)->GetWheel(chrono::vehicle::RIGHT)->GetTire()->GetSlipAngle();
            data_out[28] = m_vehicle.GetAxle(1)->GetWheel(chrono::vehicle::LEFT)->GetTire()->GetSlipAngle();
            data_out[29] = m_vehicle.GetAxle(1)->GetWheel(chrono::vehicle::RIGHT)->GetTire()->GetSlipAngle();
            data_out[30] = m_vehicle.GetDriveline()->GetSpindleTorque(0, chrono::vehicle::LEFT);
            data_out[31] = m_vehicle.GetDriveline()->GetSpindleTorque(0, chrono::vehicle::RIGHT);
            data_out[32] = m_vehicle.GetDriveline()->GetSpindleTorque(1, chrono::vehicle::LEFT);
            data_out[33] = m_vehicle.GetDriveline()->GetSpindleTorque(1, chrono::vehicle::RIGHT);
            data_out[34] = m_vehicle.GetBrake(0, chrono::vehicle::LEFT)->GetBrakeTorque();
            data_out[35] = m_vehicle.GetBrake(0, chrono::vehicle::RIGHT)->GetBrakeTorque();
            data_out[36] = m_vehicle.GetBrake(1, chrono::vehicle::LEFT)->GetBrakeTorque();
            data_out[37] = m_vehicle.GetBrake(1, chrono::vehicle::RIGHT)->GetBrakeTorque();
            data_out[38] = m_inputs.m_steering / m_vehicle.GetMaxSteeringAngle();
            
            return data_out;
        }
    
    private:
        chrono::vehicle::WheeledVehicleForce m_vehicle;
        std::shared_ptr<chrono::vehicle::ChPowertrainAssembly> m_powertrain;
        chrono::vehicle::RigidTerrain m_terrain{m_vehicle.GetSystem(), chrono::vehicle::GetDataFile("terrain/RigidPlane.json")};
        chrono::vehicle::DriverInputs m_inputs{0.0, 0.0, 0.0};
    };
}

PYBIND11_MODULE(vehicle_interface, m) {
    py::class_<chrono::VehicleInterface>(m, "VehicleInterface")
        .def(py::init<const std::string&, const std::string&, const std::string&, const std::vector<std::string>&, chrono::ChContactMethod>())
        .def("set_inputs", &chrono::VehicleInterface::SetInputs)
        .def("advance", &chrono::VehicleInterface::Advance)
        .def("get_state", &chrono::VehicleInterface::GetState);
}
