// Utilities that mirror Project Chrono's Vehicle Utilities. This is intended to
// replace ReadSteeringJSON.

#include "src/steering/RackPinionForce.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include <memory>
#include <string>

namespace chrono
{
  namespace vehicle
  {
    /// @brief Creates a steering system from a JSON file. This also supports
    /// creating a force-based steering system.
    /// @param filename the json filename.
    /// @return a pointer to the created steering system.
    std::shared_ptr<ChSteering> ReadForceSteeringJSON(const std::string &filename)
    {
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
      if (subtype.compare("PitmanArm") == 0)
      {
        steering = chrono_types::make_shared<PitmanArm>(d);
      }
      else if (subtype.compare("RackPinion") == 0)
      {
        steering = chrono_types::make_shared<RackPinion>(d);
      }
      else if (subtype.compare("RackPinionForce") == 0)
      {
        steering = chrono_types::make_shared<RackPinionForce>(d);
      }
      else if (subtype.compare("RotaryArm") == 0)
      {
        steering = chrono_types::make_shared<RotaryArm>(d);
      }
      else
      {
        throw std::runtime_error("Steering type not supported in ReadSteeringJSON.");
      }

      return steering;
    }

  } // namespace vehicle
} // namespace chrono