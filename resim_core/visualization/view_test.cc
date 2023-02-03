#include "resim_core/visualization/view.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <thread>
#include <utility>
#include <variant>

#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/view_client.hh"

namespace resim::visualization {
namespace {

// A simple mock of the view client that calls the observer given on
// construction when send_view_update is called.
class MockViewClient : public ViewClient {
 public:
  using Observer = std::function<void(const ViewUpdate &)>;

  explicit MockViewClient(Observer &&observer)
      : observer_{std::move(observer)} {}

  Status send_view_update(const ViewUpdate &update) override {
    observer_(update);
    return should_succeed_ ? OKAY_STATUS : MAKE_STATUS("Fail!");
  }

  void set_should_succeed(const bool should_succeed) {
    should_succeed_ = false;
  };

 private:
  Observer observer_;
  bool should_succeed_ = true;
};

using transforms::SE3;

}  // namespace

TEST(ViewTest, TestViewSingleThread) {
  // SETUP
  constexpr std::size_t NUM_SE3S = 100;
  std::vector<SE3> test_elements{
      transforms::make_test_group_elements<SE3>(NUM_SE3S)};
  std::vector<SE3> result_elements;

  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {
        // expect one element at a time
        EXPECT_EQ(update.primitives.size(), 1U);
        const ViewPrimitive prim{update.primitives.front()};
        ASSERT_TRUE(std::holds_alternative<SE3>(prim.payload));
        result_elements.push_back(std::get<SE3>(prim.payload));
      });
  view.set_client(std::move(mock_client));

  // ACTION
  for (const SE3 &se3 : test_elements) {
    view << se3;
  }

  // VERIFICATION
  ASSERT_EQ(result_elements.size(), test_elements.size());
  for (std::size_t ii = 0U; ii < test_elements.size(); ++ii) {
    EXPECT_TRUE(result_elements.at(ii).is_approx(test_elements.at(ii)));
  }
}

TEST(ViewTest, TestViewMultiThread) {
  // SETUP
  // This number consistently causes failures in the absence of mutexes
  constexpr std::size_t NUM_SE3S = 100;
  std::vector<SE3> test_elements{
      transforms::make_test_group_elements<SE3>(NUM_SE3S)};

  std::vector<SE3> result_elements;
  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {
        for (const auto &prim : update.primitives) {
          ASSERT_TRUE(std::holds_alternative<SE3>(prim.payload));
          result_elements.push_back(std::get<SE3>(prim.payload));
        }
      });
  view.set_client(std::move(mock_client));

  // ACTION
  std::vector<std::thread> threads;
  threads.reserve(test_elements.size());
  for (const auto &test_element : test_elements) {
    threads.emplace_back([&test_element]() { view << test_element; });
  }
  for (auto &thread : threads) {
    thread.join();
  }

  // VERIFICATION
  ASSERT_EQ(result_elements.size(), test_elements.size());

  // Sort the SE3 element vectors into the same order since they will likely be
  // jumbled.
  const auto &sorter = [](auto &&vector) {
    std::sort(vector.begin(), vector.end(), [](const auto &a, const auto &b) {
      return a.log().x() < b.log().x();
    });
  };
  sorter(test_elements);
  sorter(result_elements);
  for (std::size_t ii = 0U; ii < test_elements.size(); ++ii) {
    EXPECT_TRUE(test_elements.at(ii).is_approx(result_elements.at(ii)));
  }
}

TEST(ViewTest, TestDestructorCoverage) {
  // Since we're a friend test, we can get test coverage of the destructor which
  // GCOV insists on like so. One would never create a View in this way in any
  // production code as it is a Singleton.
  EXPECT_NO_THROW({
    View local_view;
    (void)local_view;
  });
}

// Test that only a single instance of View exists:
TEST(ViewTest, TestSingleInstance) { EXPECT_EQ(&View::get_instance(), &view); }

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(ViewDeathTest, TestFailedSend) {
  // SETUP
  std::vector<SE3> test_elements{transforms::make_test_group_elements<SE3>()};

  // Make a mock client that will fail to send the update:
  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {});
  mock_client->set_should_succeed(false);

  view.set_client(std::move(mock_client));

  // ACTION / VERIFICATION
  EXPECT_DEATH(view << test_elements.front(), "Fail!");
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::visualization
