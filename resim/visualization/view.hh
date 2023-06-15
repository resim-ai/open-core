#pragma once

#ifdef RESIM_TESTING
#include <gtest/gtest_prod.h>
#endif

#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "resim/visualization/view_primitive.hh"

namespace resim {
namespace visualization {

// Forward declaration of the ViewClient interface.
class ViewClientInterface;

// This class holds the data that a view statement can generate and acts
// as the implementation of the logic behind the desired synax:
//
// VIEW(type) << "the_name" // Visualize the type with name "the_name".
// VIEW(type,"name")        // Visualize the type with name "the_name".
// resim::view << type      // Visualize without a custom name.
//
// It does this by implementing a streaming operator on the ViewObject class
// which constructs and flushes a view primitive. The class is declared within
// the view header, as they are intimately coupled.
//
// NOTE: it is currently not possible to combine streaming operators, for
// example:
//
// VIEW(type) << "the_" << "name"
//
//  will not work as expected as the operator has a `void` return type.
template <typename T>
struct ViewObject {
  // The object to visualize
  T the_object;
  // The optional name provided for the object
  std::optional<std::string> user_defined_name;
  // The file name of the source file where the view statement was placed
  const char *file_name;
  // The line number of the source file where the view statement was placed
  const int line_number;
  // Constructor
  // Creates an, as of yet unnamed ViewObject: needs to be combined with the
  // stream operator to generate a view. Includes the file name and line number
  // where the view object was generated.
  ViewObject(T object, const char *file_name, int line_number);
  // Constructor
  // Creates and flushes a view object eagerly, alongside the file and line
  // number where it was generated.
  ViewObject(
      T object,
      std::string user_defined_name,
      const char *file_name,
      int line_number);
  // Streams in a name for the object. Returns void as
  // we do not allow composite names with a streaming operator.
  void operator<<(const std::string &user_defined_name);
};

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

  // Generates and flushes a view primitive from a view object
  template <typename T>
  void view_object(ViewObject<T> view_object);

  // Transmit all currently stored primitives to the server via the current
  // client. Fails if the client has not been set.
  void flush();

 private:
#ifdef RESIM_TESTING
  template <typename T>
  FRIEND_TEST(ViewClientTest, TestViewClientView);

  template <typename T>
  FRIEND_TEST(ViewClientTest, TestViewClientViewCustomName);

  template <typename T>
  FRIEND_TEST(ViewClientTest, TestViewClientViewCustomNameAlt);

  template <typename T>
  FRIEND_TEST(ViewClientTest, TestViewClientLogging);

  template <typename T>
  FRIEND_TEST(ViewTest, TestViewSingleThread);

  template <typename T>
  FRIEND_TEST(ViewTest, TestViewMultiThread);

  template <typename T>
  FRIEND_TEST(ViewTest, TestDestructorCoverage);

  template <typename T>
  FRIEND_TEST(ViewTest, TestFailedSend);

  FRIEND_TEST(ViewIntegrationTest, ViewExecutesAndLogs);

  template <typename T>
  FRIEND_TEST(ViewObjectTest, TestViewObjectNamedConstructor);

#endif

  // Default constructor used by get_instance() above.
  View();

  // Set the client pointer
  // @param[in] The new client.
  void set_client(std::unique_ptr<ViewClientInterface> &&client);

  std::unique_ptr<ViewClientInterface> client_;

  std::vector<ViewPrimitive> primitives_;
  std::mutex primitives_mutex_;
};

// Implementation of the VIEW macro, without a message. Currently just calls the
// constructor for a view object.
template <typename T>
ViewObject<T> view_impl(const T &object, const char *file, const int line) {
  return ViewObject<T>(object, file, line);
}

// Implementation of the VIEW macro, with a message. Currently just calls the
// constructor for a view object.
template <typename T>
ViewObject<T> view_impl(
    const T &object,
    std::string message,
    const char *file,
    const int line) {
  return ViewObject<T>(object, message, file, line);
}

}  // namespace visualization

// Global variable to access singleton instance of View.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static visualization::View &view{visualization::View::get_instance()};

}  // namespace resim

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
// Helper macro to help us select the right "overload" of the macro for cases
// when the user decides not to pass a message to the view using a streaming
// operator.
#define SELECT_VIEWER(_1, _2, VIEWER, ...) VIEWER

// "Overload" for standard case when no message is provided and must be streamed
// in:
#define VIEW_1(object) \
  resim::visualization::view_impl(object, __FILE__, __LINE__)

// "Overload" for the case where a message is provided in macro.
#define VIEW_2(object, message) \
  resim::visualization::view_impl(object, message, __FILE__, __LINE__)

// The main VIEW macro that users should use.
#define VIEW(...) SELECT_VIEWER(__VA_ARGS__, VIEW_2, VIEW_1)(__VA_ARGS__)
// NOLINTEND(cppcoreguidelines-macro-usage)
