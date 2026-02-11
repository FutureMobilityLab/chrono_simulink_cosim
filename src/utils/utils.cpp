// Utilities that mirror Project Chrono's Vehicle Utilities. This is intended to
// replace ReadSteeringJSON.

#include "src/utils/utils.h"
#include "src/steering/RackPinionForce.h"
#include "src/tire/SalaaniTire.h"
#include "src/tire/StiremodTire.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include <memory>
#include <string>

namespace chrono::vehicle {

  std::shared_ptr<ChSteering> ReadForceSteeringJSON(const std::string &filename) {
    std::shared_ptr<ChSteering> steering;

    rapidjson::Document d;
    chrono::vehicle::ReadFileJSON(filename, d);
    if (d.IsNull())
      return nullptr;

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Steering") == 0);

    // Extract the steering type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("PitmanArm") == 0) {
      steering = chrono_types::make_shared<PitmanArm>(d);
    } else if (subtype.compare("RackPinion") == 0) {
      steering = chrono_types::make_shared<RackPinion>(d);
    } else if (subtype.compare("RackPinionForce") == 0) {
      steering = chrono_types::make_shared<RackPinionForce>(d);
    } else if (subtype.compare("RotaryArm") == 0){
      steering = chrono_types::make_shared<RotaryArm>(d);
    } else {
      throw std::runtime_error("Steering type not supported in ReadSteeringJSON.");
    }

    return steering;
  }

  std::shared_ptr<ChTire> ReadCustomTireJSON(const std::string& filename) {
    std::shared_ptr<ChTire> tire;

    rapidjson::Document d;
    std::cout << "Reading JSON: '" << filename << "'\n";
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a tire specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Tire") == 0);

    // Extract the tire type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();
  
    // Create the tire using the appropriate template.
    if (subtype.compare("RigidTire") == 0) {
        tire = chrono_types::make_shared<RigidTire>(d);
    } else if (subtype.compare("TMeasyTire") == 0) {
        tire = chrono_types::make_shared<TMeasyTire>(d);
    } else if (subtype.compare("TMsimpleTire") == 0) {
        tire = chrono_types::make_shared<TMsimpleTire>(d);
    } else if (subtype.compare("FialaTire") == 0) {
        tire = chrono_types::make_shared<FialaTire>(d);
    } else if (subtype.compare("Pac89Tire") == 0) {
        tire = chrono_types::make_shared<Pac89Tire>(d);
    } else if (subtype.compare("Pac02Tire") == 0) {
        tire = chrono_types::make_shared<Pac02Tire>(d);
    } else if (subtype.compare("ANCFTire") == 0) {
        tire = chrono_types::make_shared<ANCFTire>(d);
    } else if (subtype.compare("ReissnerTire") == 0) {
        tire = chrono_types::make_shared<ReissnerTire>(d);
    } else if (subtype.compare("FEATire") == 0) {
        tire = chrono_types::make_shared<FEATire>(d);
    } else if (subtype.compare("SalaaniTire") == 0) {
        tire = chrono_types::make_shared<SalaaniTire>(d);
    } else if (subtype.compare("StiremodTire") == 0) {
        tire = chrono_types::make_shared<StiremodTire>(d);
    } else {
        throw std::invalid_argument("Tire type not supported in ReadTireJSON.");
    }

    return tire;
  }

  
  void CheckHasMember(const rapidjson::Document& d, const std::string& member_name) {
    if (!d.HasMember(member_name.c_str())) {
      throw std::runtime_error("Missing required member '" + member_name + "' in JSON file");
    }
  }

  void CheckHasMember(const rapidjson::Value::ConstObject& d, const std::string& member_name) {
    if (!d.HasMember(member_name.c_str())) {
      throw std::runtime_error("Missing required member '" + member_name + "' in JSON file");
    }
  }


} // namespace chrono::vehicle