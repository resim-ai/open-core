#pragma once

#include <gtest/gtest_prod.h>

#include <mutex>
#include <utility>

#include "resim_core/visualization/view_client.hh"
#include "resim_core/visualization/view_primitive.hh"

namespace resim {
namespace visualization {

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
  ~View() = default;

  // Getter for the single instance of this Singleton
  static View &get_instance();

  // Streaming operator which accepts any type that can be held in a
  // ViewPrimitive and stores these internally.
  template <typename T>
  View &operator<<(T &&subject);

  // Transmit all currently stored primitives to the server via the current
  // client. Fails if the client has not been set.
  void flush();

 private:
  FRIEND_TEST(LibcurlClientTest, TestLibcurlClientView);
  FRIEND_TEST(ViewTest, TestViewSingleThread);
  FRIEND_TEST(ViewTest, TestViewMultiThread);
  FRIEND_TEST(ViewTest, TestDestructorCoverage);
  FRIEND_TEST(ViewDeathTest, TestFailedSend);

  // Default constructor used by get_instance() above.
  View();

  // Set the client pointer
  // @param[in] The new client.
  void set_client(std::unique_ptr<ViewClient> &&client);

  std::unique_ptr<ViewClient> client_;

  std::vector<ViewPrimitive> primitives_;
  std::mutex primitives_mutex_;
};

template <typename T>
View &View::operator<<(T &&subject) {
  {
    // In a separate scope since flush() needs this lock
    std::lock_guard<std::mutex> guard{primitives_mutex_};

    // We simply push the primitivies in here without packing
    // them. The whole ViewUpdate will be packed by the ViewClient.
    primitives_.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = std::forward<T>(subject),
    });
  }
  // Currently, we pack each primitive into its own update and send eagerly to
  // the view server.
  flush();
  return *this;
}

}  // namespace visualization

// Global variable to access singleton instance of View.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static visualization::View &view{visualization::View::get_instance()};

}  // namespace resim
