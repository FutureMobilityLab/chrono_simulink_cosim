#ifndef UTILS_H
#define UTILS_H

#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include <memory>

namespace chrono::vehicle {

  std::shared_ptr<ChSteering> ReadForceSteeringJSON(const std::string &filename);

  std::shared_ptr<ChTire> ReadCustomTireJSON(const std::string& filename);

}

#endif