#ifndef UTILS_H
#define UTILS_H

#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include <memory>

namespace chrono::vehicle {

  /// @brief Creates a steering system from a JSON file. This also supports
  /// creating a force-based steering system.
  /// @param filename the json filename.
  /// @return a pointer to the created steering system.
  std::shared_ptr<ChSteering> ReadForceSteeringJSON(const std::string &filename);

  std::shared_ptr<ChTire> ReadCustomTireJSON(const std::string& filename);

  void CheckHasMember(const rapidjson::Document& d, const std::string& member_name);
  void CheckHasMember(const rapidjson::Value::ConstObject& d, const std::string& member_name);

} // namespace chrono::vehicle

#endif