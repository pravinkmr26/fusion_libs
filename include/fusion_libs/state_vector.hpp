#pragma once

namespace fusion::utils::state {

typedef unsigned long size_t;

struct StateVector {
  static constexpr size_t X = 0;
  static constexpr size_t Y = 1;
  static constexpr size_t Yaw = 2;
  static constexpr size_t Veloicy = 3;
  static constexpr size_t YawRate = 4;
  static constexpr size_t Size = 5;
};
} // namespace fusion::utils::state