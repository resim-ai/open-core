
#include "resim/converter/fuzz_helpers.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/utils/inout.hh"

namespace resim::geometry {

template <typename Rng>
Wireframe random_element(
    converter::TypeTag<Wireframe> /*unused*/,
    InOut<Rng> rng) {
  auto points = converter::random_element<std::vector<Eigen::Vector3d>>(rng);

  auto indices =
      converter::random_element<std::vector<std::array<std::size_t, 2>>>(rng);

  for (auto &pair : indices) {
    for (auto &idx : pair) {
      idx %= points.size();
    }
    // Make sure the points are distinct
    if (pair.at(0) == pair.at(1)) {
      pair.at(1) += 1;
      pair.at(1) %= points.size();
    }
  }
  return Wireframe(points, indices);
}

bool custom_verify_equality(const Wireframe &a, const Wireframe &b);

}  // namespace resim::geometry
