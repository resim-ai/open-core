#include "resim_core/visualization/view.hh"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <variant>

#include "resim_core/assert/assert.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/testing/test_directory.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/client/view_client_libcurl.hh"
#include "resim_core/visualization/testing/mock_server.hh"
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

// Get a vector of arbitrary random SE3s of a given size.
// @param[in] num_se3s - The number of se3s to get.
std::vector<SE3> random_se3s(const std::size_t num_se3s) {
  constexpr unsigned SEED = 8943U;
  std::mt19937 rng{SEED};

  std::vector<SE3> result;
  result.reserve(num_se3s);
  for (std::size_t ii = 0U; ii < num_se3s; ++ii) {
    result.push_back(
        SE3::exp(resim::testing::random_vector<SE3::TangentVector>(rng)));
  }

  return result;
}

}  // namespace

TEST(LibcurlClientTest, TestClientBasicFunction) {
  // SET UP
  testing::MockServer server{"localhost", UUID::new_uuid(), [](auto &&...) {}};
  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));

  // Create SE3 objects and corresponding ViewUpdates.
  constexpr std::size_t NUM_SE3S = 100;
  std::vector<SE3> test_elements{random_se3s(NUM_SE3S)};
  ViewUpdate update;
  for (const SE3 &se3 : test_elements) {
    update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = se3,
    });
  }

  // ACTION
  Status status = mock_client->send_view_update(update);
  REASSERT(status.ok());

  status = mock_client->send_view_update(update);
  REASSERT(status.ok());

  status = mock_client->send_view_update(update);
  REASSERT(status.ok());
}

TEST(LibcurlClientTest, TestClientBasicFunctionFail) {
  // SET UP
  testing::MockServer server{
      "localhost",
      UUID::new_uuid(),
      [](auto &&...) {},
      testing::MockServer::ResponseCode::NOT_FOUND};

  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));

  // Create SE3 objects and corresponding ViewUpdates.
  constexpr std::size_t NUM_SE3S = 100;
  std::vector<SE3> test_elements{random_se3s(NUM_SE3S)};
  ViewUpdate update;
  for (const SE3 &se3 : test_elements) {
    update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = se3,
    });
  }

  // ACTION
  Status status = mock_client->send_view_update(update);
  REASSERT(!status.ok());
}

TEST(LibcurlClientTest, TestFail) {
  // Do not set up a server
  auto mock_client = std::make_unique<LibcurlClient>("zzzz");
  ViewUpdate update;

  // ACTION
  EXPECT_THROW(mock_client->send_view_update(update), AssertException);
}

TEST(LibcurlClientTest, TestLibcurlClientView) {
  // SET UP
  ViewUpdate expected_update;
  const UUID expected_session_id{UUID::new_uuid()};
  uint64_t expected_update_id = 0;

  // Create SE3 objects and corresponding ViewUpdates.
  constexpr std::size_t NUM_SE3S = 100;
  std::vector<SE3> test_elements{random_se3s(NUM_SE3S)};
  for (const SE3 &se3 : test_elements) {
    expected_update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = se3,
    });
  }

  // Create MockServer with a receiver function to verify correctness.
  testing::MockServer server{
      "localhost",
      expected_session_id,
      [&](const ViewUpdate &update,
          const UUID &session_id,
          const uint64_t update_id) {
        EXPECT_EQ(session_id, expected_session_id);
        EXPECT_EQ(update_id, expected_update_id);

        // Since we feed views one at a time, update.primitives will always only
        // have one element, but it should correspond with the expected_update
        // at index update_id.
        EXPECT_TRUE(std::get<SE3>(update.primitives.at(0).payload)
                        .is_approx(std::get<SE3>(
                            expected_update.primitives.at(update_id).payload)));
      }};

  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  view.set_client(std::move(mock_client));

  // ACTION & VERIFICATION
  for (const SE3 &se3 : test_elements) {
    view << se3;
    expected_update_id++;
  }
}

TEST(LibcurlClientTest, TestLibcurlClientLogging) {
  // SETUP
  //  Create a separate test instance of glog.
  google::InitGoogleLogging("test_logging");
  // Ask glog to write logs to a temporary file.
  const resim::testing::TestDirectoryRAII tmp_log_dir;
  const auto logfile = tmp_log_dir.test_file_path();
  google::SetLogDestination(google::GLOG_INFO, logfile.string().data());
  // Setup a minimal mock server.
  testing::MockServer server{"localhost", UUID::new_uuid(), [](auto &&...) {}};
  // Setup a minimal mock client.
  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  view.set_client(std::move(mock_client));

  // ACTION
  // Send an identity SE3 to view.
  view << SE3::identity();
  // Shutdown the logger here becase A) we don't need it. B) We want to be sure
  // that the log is flushed to the logfile before we try to read it.
  google::ShutdownGoogleLogging();

  // VERIFICATION
  // We expect part of the url to be somewhere in the logfile.
  constexpr std::string_view url_part = "app.resim.ai/views";
  bool url_found = false;
  // Glog mangles logfile paths by adding timestamps. However the directory
  // is also temporary so we can simply search all (one) files in the directory.
  for (const auto &file :
       std::filesystem::directory_iterator(tmp_log_dir.path())) {
    std::ifstream file_str(file.path(), std::ios::in);
    if (!file_str.is_open()) {
      ASSERT_TRUE(false) << "Falied to open the test log file";
    } else {
      std::string line;
      while (std::getline(file_str, line)) {
        if (line.find(url_part) != std::string::npos) {
          url_found = true;
        }
      }
      file_str.close();
    }
  }
  EXPECT_TRUE(url_found);
}

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
  const std::vector<SE3> test_elements{
      transforms::make_test_group_elements<SE3>()};

  // Make a mock client that will fail to send the update:
  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {});
  mock_client->set_should_succeed(false);

  view.set_client(std::move(mock_client));

  // ACTION / VERIFICATION
  EXPECT_THROW(view << test_elements.front(), AssertException);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::visualization
