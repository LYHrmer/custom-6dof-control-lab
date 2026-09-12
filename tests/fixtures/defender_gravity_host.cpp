// Offline harness drafted by Claude Opus and corrected against the actual API.
// The only linked reference translation unit is arm_gravity.cpp: no board I/O.
#include "arm_gravity.h"
#include "arm_gravity_fitted_model.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

int main() {
  using namespace sentry::arm;
  static_assert(kArmFittedJointCount == 6, "this harness is six-axis only");
  ArmGravityUrdfConfig config{};
  config.jointCount = kArmFittedJointCount;
  for (int i = 0; i < 6; ++i) config.joint[i] = kArmFittedJointRows[i];
  for (int i = 0; i < 3; ++i) {
    config.toolTranslation[i] = kArmFittedToolTranslation[i];
    config.toolRpy[i] = kArmFittedToolRpy[i];
  }
  ArmGravityCompensator compensator;
  if (!compensator.configureUrdf(config)) return 1;
  compensator.setParameters(kArmFittedPhi);
  std::cout.precision(std::numeric_limits<float>::max_digits10);

  std::string line;
  unsigned rows = 0;
  while (std::getline(std::cin, line)) {
    std::istringstream stream(line);
    double values[9];
    for (double& value : values) {
      if (!(stream >> value) || !std::isfinite(value)) {
        std::cerr << "invalid input row " << rows + 1 << '\n';
        return 1;
      }
    }
    std::string extra;
    const double norm = std::hypot(values[0], values[1], values[2]);
    if ((stream >> extra) || !std::isfinite(norm) || norm == 0.0) {
      std::cerr << "expected 9 finite values and a nonzero up vector\n";
      return 1;
    }
    const float up[3] = {static_cast<float>(values[0] / norm),
                         static_cast<float>(values[1] / norm),
                         static_cast<float>(values[2] / norm)};
    float q[6], tau[6]{};
    for (int i = 0; i < 6; ++i) {
      q[i] = static_cast<float>(values[i + 3]);
      if (!std::isfinite(q[i])) return 1;
    }
    compensator.setGravityDirection(up);
    compensator.computeTorques(q, tau);
    for (float value : tau) if (!std::isfinite(value)) return 1;
    for (int i = 0; i < 6; ++i) std::cout << tau[i] << (i == 5 ? '\n' : ' ');
    ++rows;
  }
  if (std::cin.bad() || rows == 0) {
    std::cerr << "empty input or stream error\n";
    return 1;
  }
  return 0;
}
