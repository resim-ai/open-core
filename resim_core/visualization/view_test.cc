#define RESIM_TESTING

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

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/auth/testing/mock_auth_client.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/testing/test_directory.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/client/view_client_libcurl.hh"
#include "resim_core/visualization/curve/test_helpers.hh"
#include "resim_core/visualization/testing/mock_server.hh"
#include "resim_core/visualization/view_client.hh"
#include "resim_core/visualization/view_server/view_server_test_helper.hh"

using ::resim::visualization::client::proto::ViewSessionUpdateResponse;

namespace resim::visualization {
namespace {

using transforms::FSE3;
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;

constexpr unsigned int NUM_PAYLOADS = 10;

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
}  // namespace

template <typename T>
class LibcurlClientTest : public ::testing::Test {
 public:
  static void check_correctness(const T &original, const T &expected);
};

template <>
void LibcurlClientTest<Frame>::check_correctness(
    const Frame &original,
    const Frame &expected) {
  EXPECT_TRUE(original == expected);  // Using Frame equality
}

template <>
void LibcurlClientTest<SE3>::check_correctness(
    const SE3 &original,
    const SE3 &expected) {
  EXPECT_TRUE(original.is_approx(expected));
}

template <>
void LibcurlClientTest<SO3>::check_correctness(
    const SO3 &original,
    const SO3 &expected) {
  EXPECT_TRUE(original.is_approx(expected));
}

template <>
void LibcurlClientTest<FSE3>::check_correctness(
    const FSE3 &original,
    const FSE3 &expected) {
  EXPECT_TRUE(original.is_approx(expected));
}

template <>
void LibcurlClientTest<FSO3>::check_correctness(
    const FSO3 &original,
    const FSO3 &expected) {
  EXPECT_TRUE(original.is_approx(expected));
}

template <>
void LibcurlClientTest<curves::DCurve<SE3>>::check_correctness(
    const curves::DCurve<SE3> &original,
    const curves::DCurve<SE3> &expected) {
  const auto &orig_ctrl_pts = original.control_pts();
  const auto &test_ctrl_pts = expected.control_pts();

  ASSERT_EQ(orig_ctrl_pts.size(), test_ctrl_pts.size());

  for (int i = 0; i < orig_ctrl_pts.size(); i++) {
    const auto orig_ref_from_ctrl = orig_ctrl_pts.at(i).ref_from_control;
    const auto test_ref_from_ctrl = test_ctrl_pts.at(i).ref_from_control;
    EXPECT_TRUE(orig_ref_from_ctrl->is_approx(*test_ref_from_ctrl));
  }
}

template <>
void LibcurlClientTest<curves::DCurve<FSE3>>::check_correctness(
    const curves::DCurve<FSE3> &original,
    const curves::DCurve<FSE3> &expected) {
  const auto &orig_ctrl_pts = original.control_pts();
  const auto &test_ctrl_pts = expected.control_pts();

  ASSERT_EQ(orig_ctrl_pts.size(), test_ctrl_pts.size());

  for (int i = 0; i < orig_ctrl_pts.size(); i++) {
    const auto orig_ref_from_ctrl = orig_ctrl_pts.at(i).ref_from_control;
    const auto test_ref_from_ctrl = test_ctrl_pts.at(i).ref_from_control;
    EXPECT_TRUE(orig_ref_from_ctrl->is_approx(*test_ref_from_ctrl));
  }
}

template <>
void LibcurlClientTest<curves::TCurve<FSE3>>::check_correctness(
    const curves::TCurve<FSE3> &original,
    const curves::TCurve<FSE3> &expected) {
  const auto &orig_ctrl_pts = original.control_pts();
  const auto &test_ctrl_pts = expected.control_pts();

  ASSERT_EQ(orig_ctrl_pts.size(), test_ctrl_pts.size());

  for (int i = 0; i < orig_ctrl_pts.size(); i++) {
    const auto &orig_ctrl_pt = orig_ctrl_pts.at(i);
    const auto &test_ctrl_pt = test_ctrl_pts.at(i);
    EXPECT_EQ(orig_ctrl_pt.time, test_ctrl_pt.time);
    EXPECT_TRUE(orig_ctrl_pt.point.is_approx(test_ctrl_pt.point));
  }
}

template <>
void LibcurlClientTest<actor::state::Trajectory>::check_correctness(
    const actor::state::Trajectory &original,
    const actor::state::Trajectory &expected) {
  EXPECT_EQ(original.start_time(), expected.start_time());

  const auto &orig_ctrl_pts = original.curve().control_pts();
  const auto &test_ctrl_pts = expected.curve().control_pts();

  ASSERT_EQ(orig_ctrl_pts.size(), test_ctrl_pts.size());

  for (int i = 0; i < orig_ctrl_pts.size(); i++) {
    const auto &orig_ctrl_pt = orig_ctrl_pts.at(i);
    const auto &test_ctrl_pt = test_ctrl_pts.at(i);
    EXPECT_EQ(orig_ctrl_pt.time, test_ctrl_pt.time);
    EXPECT_TRUE(orig_ctrl_pt.point.is_approx(test_ctrl_pt.point));
  }
}

using PayloadTypes = ::testing::Types<
    SE3,
    SO3,
    FSE3,
    FSO3,
    curves::DCurve<SE3>,
    curves::DCurve<FSE3>,
    curves::TCurve<FSE3>,
    actor::state::Trajectory,
    Frame>;

TYPED_TEST_SUITE(LibcurlClientTest, PayloadTypes);

TYPED_TEST(LibcurlClientTest, TestClientBasicFunction) {
  // SET UP
  testing::MockServer server{"localhost", UUID::new_uuid(), [](auto &&...) {
                               return ViewSessionUpdateResponse{};
                             }};
  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  mock_client->set_auth_client(std::make_unique<auth::MockAuthClient>());

  // Create objects and corresponding ViewUpdates.
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};
  ViewUpdate update;
  for (const auto &element : test_elements) {
    update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = element,
    });
  }

  // ACTION
  Status status = mock_client->send_view_update(update);
  EXPECT_TRUE(status.ok());
}

TYPED_TEST(LibcurlClientTest, TestClientBasicFunctionFail) {
  // SET UP
  testing::MockServer server{
      "localhost",
      UUID::new_uuid(),
      [](auto &&...) { return ViewSessionUpdateResponse{}; },
      HttpResponse::NOT_FOUND};
  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  mock_client->set_auth_client(std::make_unique<auth::MockAuthClient>());

  // Create objects and corresponding ViewUpdates.
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};
  ViewUpdate update;
  for (const auto &element : test_elements) {
    update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = element,
    });
  }

  // ACTION
  Status status = mock_client->send_view_update(update);
  EXPECT_FALSE(status.ok());
}

TYPED_TEST(LibcurlClientTest, TestFail) {
  // Do not set up a server
  auto mock_client = std::make_unique<LibcurlClient>("zzzz");
  mock_client->set_auth_client(std::make_unique<auth::MockAuthClient>());

  ViewUpdate update;

  // ACTION
  Status status = mock_client->send_view_update(update);
  EXPECT_FALSE(status.ok());
}

TYPED_TEST(LibcurlClientTest, TestLibcurlClientView) {
  // SET UP
  ViewUpdate expected_update;
  const UUID expected_session_id{UUID::new_uuid()};
  uint64_t expected_update_id = 0;

  // Create Group objects and corresponding ViewUpdates.
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};
  for (const auto &element : test_elements) {
    expected_update.primitives.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = element,
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
        match(
            update.primitives.at(0).payload,
            [&](const Frame &test_frame) {
              LibcurlClientTest<Frame>::check_correctness(
                  test_frame,
                  std::get<Frame>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const SE3 &test_se3) {
              LibcurlClientTest<SE3>::check_correctness(
                  test_se3,
                  std::get<SE3>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const SO3 &test_so3) {
              LibcurlClientTest<SO3>::check_correctness(
                  test_so3,
                  std::get<SO3>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const FSE3 &test_fse3) {
              LibcurlClientTest<FSE3>::check_correctness(
                  test_fse3,
                  std::get<FSE3>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const FSO3 &test_fso3) {
              LibcurlClientTest<FSO3>::check_correctness(
                  test_fso3,
                  std::get<FSO3>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const curves::DCurve<SE3> &test_d_curve_se3) {
              LibcurlClientTest<curves::DCurve<SE3>>::check_correctness(
                  test_d_curve_se3,
                  std::get<curves::DCurve<SE3>>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const curves::DCurve<FSE3> &test_d_curve_fse3) {
              LibcurlClientTest<curves::DCurve<FSE3>>::check_correctness(
                  test_d_curve_fse3,
                  std::get<curves::DCurve<FSE3>>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const curves::TCurve<FSE3> &test_t_curve_fse3) {
              LibcurlClientTest<curves::TCurve<FSE3>>::check_correctness(
                  test_t_curve_fse3,
                  std::get<curves::TCurve<FSE3>>(
                      expected_update.primitives.at(update_id).payload));
            },
            [&](const actor::state::Trajectory &test_trajectory) {
              LibcurlClientTest<actor::state::Trajectory>::check_correctness(
                  test_trajectory,
                  std::get<actor::state::Trajectory>(
                      expected_update.primitives.at(update_id).payload));
            });
        return ViewSessionUpdateResponse{};
      }};

  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  mock_client->set_auth_client(std::make_unique<auth::MockAuthClient>());
  view.set_client(std::move(mock_client));

  // ACTION & VERIFICATION
  for (const auto &element : test_elements) {
    view << element;
    expected_update_id++;
  }
}

TYPED_TEST(LibcurlClientTest, TestLibcurlClientLogging) {
  // SETUP
  //  Create a separate test instance of glog.
  google::InitGoogleLogging("test_logging");
  // Ask glog to write logs to a temporary file.
  const resim::testing::TestDirectoryRAII tmp_log_dir;
  const auto logfile = tmp_log_dir.test_file_path();
  google::SetLogDestination(google::GLOG_INFO, logfile.string().data());
  // Setup a minimal mock server.
  testing::MockServer server{"localhost", UUID::new_uuid(), [](auto &&...) {
                               ViewSessionUpdateResponse response;
                               response.set_view("app.resim.ai/view");
                               return response;
                             }};
  // Setup a minimal mock client.
  auto mock_client = std::make_unique<LibcurlClient>(
      fmt::format("localhost:{}", server.port()));
  mock_client->set_auth_client(std::make_unique<auth::MockAuthClient>());
  view.set_client(std::move(mock_client));

  // ACTION
  // Send viewable types to view.
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};

  REASSERT(!test_elements.empty(), "Did not generate test elements!");
  view << test_elements[0];

  // Shutdown the logger here becase A) we don't need it. B) We want to be sure
  // that the log is flushed to the logfile before we try to read it.
  google::ShutdownGoogleLogging();

  // VERIFICATION
  // We expect part of the url to be somewhere in the logfile.
  constexpr std::string_view url_part = "app.resim.ai/view";
  bool url_found = false;
  // Glog mangles logfile paths by adding timestamps. However the directory
  // is also temporary so we can simply search all (one) files in the directory.
  for (const auto &file :
       std::filesystem::directory_iterator(tmp_log_dir.path())) {
    std::ifstream file_str(file.path(), std::ios::in);
    if (!file_str.is_open()) {
      ASSERT_TRUE(false) << "Failed to open the test log file";
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

template <typename T>
class ViewTest : public LibcurlClientTest<T> {
 public:
  static std::unique_ptr<MockViewClient> mock_single_thread_client(
      std::vector<T> &result_elements);

  static std::unique_ptr<MockViewClient> mock_multi_thread_client(
      std::vector<T> &result_elements);

  void sort_elements(std::vector<T> &result_elements);
};

template <typename Group>
std::unique_ptr<MockViewClient> ViewTest<Group>::mock_single_thread_client(
    std::vector<Group> &result_elements) {
  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {
        // expect one element at a time
        EXPECT_EQ(update.primitives.size(), 1U);
        const ViewPrimitive prim{update.primitives.front()};
        ASSERT_TRUE(std::holds_alternative<Group>(prim.payload));
        result_elements.push_back(std::get<Group>(prim.payload));
      });

  return mock_client;
}

template <typename Group>
std::unique_ptr<MockViewClient> ViewTest<Group>::mock_multi_thread_client(
    std::vector<Group> &result_elements) {
  auto mock_client =
      std::make_unique<MockViewClient>([&](const ViewUpdate &update) {
        for (const auto &prim : update.primitives) {
          ASSERT_TRUE(std::holds_alternative<Group>(prim.payload));
          result_elements.push_back(std::get<Group>(prim.payload));
        }
      });

  return mock_client;
}

template <>
void ViewTest<Frame>::sort_elements(std::vector<Frame> &result_elements) {
  std::sort(
      begin(result_elements),
      end(result_elements),
      [](const auto &a, const auto &b) {
        return a.id().to_string() < b.id().to_string();
      });
}

template <>
void ViewTest<SE3>::sort_elements(std::vector<SE3> &result_elements) {
  std::sort(
      begin(result_elements),
      end(result_elements),
      [](const auto &a, const auto &b) { return a.log().x() < b.log().x(); });
}

template <>
void ViewTest<SO3>::sort_elements(std::vector<SO3> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) { return a.log().x() < b.log().x(); });
}

template <>
void ViewTest<FSE3>::sort_elements(std::vector<FSE3> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) { return a.log().x() < b.log().x(); });
}

template <>
void ViewTest<FSO3>::sort_elements(std::vector<FSO3> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) { return a.log().x() < b.log().x(); });
}

template <>
void ViewTest<curves::DCurve<SE3>>::sort_elements(
    std::vector<curves::DCurve<SE3>> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) {
        return a.curve_length() < b.curve_length();
      });
}

template <>
void ViewTest<curves::DCurve<FSE3>>::sort_elements(
    std::vector<curves::DCurve<FSE3>> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) {
        return a.curve_length() < b.curve_length();
      });
}

template <>
void ViewTest<curves::TCurve<FSE3>>::sort_elements(
    std::vector<curves::TCurve<FSE3>> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) { return a.end_time() < b.end_time(); });
}

template <>
void ViewTest<actor::state::Trajectory>::sort_elements(
    std::vector<actor::state::Trajectory> &result_elements) {
  std::sort(
      result_elements.begin(),
      result_elements.end(),
      [](const auto &a, const auto &b) {
        return a.curve().end_time() < b.curve().end_time();
      });
}

TYPED_TEST_SUITE(ViewTest, PayloadTypes);

TYPED_TEST(ViewTest, TestViewSingleThread) {
  // SETUP
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};
  std::vector<TypeParam> result_elements;

  auto mock_client =
      ViewTest<TypeParam>::mock_single_thread_client(result_elements);
  view.set_client(std::move(mock_client));

  // ACTION
  for (const auto &element : test_elements) {
    view << element;
  }

  // VERIFICATION
  ASSERT_EQ(result_elements.size(), test_elements.size());
  for (std::size_t ii = 0U; ii < test_elements.size(); ++ii) {
    LibcurlClientTest<TypeParam>::check_correctness(
        test_elements.at(ii),
        result_elements.at(ii));
  }
}

TYPED_TEST(ViewTest, TestViewMultiThread) {
  // SETUP
  // This number consistently causes failures in the absence of mutexes.
  std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};

  std::vector<TypeParam> result_elements;
  auto mock_client =
      ViewTest<TypeParam>::mock_multi_thread_client(result_elements);
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

  // Sort element vectors since they will likely be jumbled.
  ViewTest<TypeParam>::sort_elements(result_elements);
  ViewTest<TypeParam>::sort_elements(test_elements);

  for (std::size_t ii = 0U; ii < test_elements.size(); ++ii) {
    LibcurlClientTest<TypeParam>::check_correctness(
        test_elements.at(ii),
        result_elements.at(ii));
  }
}

TYPED_TEST(ViewTest, TestDestructorCoverage) {
  // Since we're a friend test, we can get test coverage of the destructor which
  // GCOV insists on like so. One would never create a View in this way in any
  // production code as it is a Singleton.
  EXPECT_NO_THROW({
    View local_view;
    (void)local_view;
  });
}

// Test that only a single instance of View exists:
TYPED_TEST(ViewTest, TestSingleInstance) {
  EXPECT_EQ(&View::get_instance(), &view);
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(ViewTest, TestFailedSend) {
  // SETUP
  const std::vector<TypeParam> test_elements{
      view_server::generate_payload_type<TypeParam>(NUM_PAYLOADS)};

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
