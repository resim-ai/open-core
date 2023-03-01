#pragma once

#ifdef RESIM_TESTING
#include <gtest/gtest_prod.h>
#endif

#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "resim_core/visualization/view_primitive.hh"

namespace resim {
namespace visualization {

// Forward declaration of the ViewClient interface.
class ViewClient;

// This class is a simple Singleton (using the Meyers approach) which acts as
// a main entrypoint for the ReSim view workflow. It holds a unique pointer to
// an abstract ViewClient which is set up in the default constructor of this
// class. This is done so we can swap the ViewClient out for a mock when unit
// testing this class.
class View {
 public:
  // Delete all of the constructors to guarantee that we can only produce one
  // instance of this object.
  View(const View &) = delete;
  View(View &&) = delete;
  View &operator=(const View &) = delete;
  View &operator=(View &&) = delete;
  ~View();

  // Getter for the single instance of this Singleton
  static View &get_instance();

  // Streaming operator which accepts any type that can be held in a
  // ViewPrimitive and stores these internally.
  template <typename T>
  View &operator<<(const T &subject);

  // Transmit all currently stored primitives to the server via the current
  // client. Fails if the client has not been set.
  void flush();

 private:
#ifdef RESIM_TESTING
  FRIEND_TEST(LibcurlClientTest, TestLibcurlClientView);
  FRIEND_TEST(LibcurlClientTest, TestLibcurlClientLogging);
  FRIEND_TEST(ViewTest, TestViewSingleThread);
  FRIEND_TEST(ViewTest, TestViewMultiThread);
  FRIEND_TEST(ViewTest, TestDestructorCoverage);
  FRIEND_TEST(ViewDeathTest, TestFailedSend);
#endif

  // Default constructor used by get_instance() above.
  View();

  // Set the client pointer
  // @param[in] The new client.
  void set_client(std::unique_ptr<ViewClient> &&client);

  std::unique_ptr<ViewClient> client_;

  std::vector<ViewPrimitive> primitives_;
  std::mutex primitives_mutex_;
};

}  // namespace visualization

// Global variable to access singleton instance of View.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static visualization::View &view{visualization::View::get_instance()};

}  // namespace resim
