
#include "resim/testing/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/utils/inout.hh"

namespace resim::testing {

namespace some_namespace {

struct TestStruct {
  int val = 0;
};

template <typename Rng>
TestStruct random_element(TypeTag<TestStruct> /*unused*/, InOut<Rng> rng) {
  return TestStruct{
      .val = random_element<int>(rng),
  };
}

bool verify_equality(const TestStruct &a, const TestStruct &b) {
  return a.val == b.val;
}

}  // namespace some_namespace

TEST(FuzzHelpersTest, TestRandomVector) {
  // SETUP
  std::mt19937 rng;

  const auto test_vec = random_element(
      TypeTag<std::vector<some_namespace::TestStruct>>(),
      InOut{rng});

  auto test_vec_different_size{test_vec};
  test_vec_different_size.emplace_back(
      resim::random_element<some_namespace::TestStruct>(InOut{rng}));

  auto test_vec_different_value{test_vec};
  test_vec_different_value.front() =
      resim::random_element<some_namespace::TestStruct>(InOut{rng});

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(test_vec, test_vec));

  EXPECT_FALSE(verify_equality(test_vec_different_size, test_vec));
  EXPECT_FALSE(verify_equality(test_vec_different_value, test_vec));

  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_size));
  EXPECT_FALSE(verify_equality(test_vec, test_vec_different_value));
}
};  // namespace resim::testing
