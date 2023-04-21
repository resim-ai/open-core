#pragma once

#include <optional>
#include <string>

#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

namespace detail {
constexpr auto TEST_FILE_NAME = "test_file_name";
constexpr int TEST_LINE_NUMBER = 42;
}  // namespace detail

// Generates a test object for a given resim type
template <typename T>
T generate_test_object();

// Generates a test ViewPrimitive for a given resim type
template <typename T>
ViewPrimitive generate_test_primitive(const std::optional<std::string>& name);

}  // namespace resim::visualization
