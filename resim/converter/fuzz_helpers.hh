
#pragma once

#include <google/protobuf/timestamp.pb.h>

#include <Eigen/Dense>
#include <array>
#include <concepts>
#include <random>
#include <string>
#include <variant>

#include "resim/converter/parser.hh"
#include "resim/converter/tags.hh"
#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/match.hh"

namespace resim::converter {

template <typename T, typename Rng>
T random_element(InOut<Rng> rng) {
  return random_element(TypeTag<T>(), rng);
}

template <typename T>
bool verify_equality(const T &a, const T &b) {
  return verify_equality(ADLTag(), a, b);
}

template <std::equality_comparable T>
bool verify_equality(ADLTag /*unused*/, const T &a, const T &b) {
  return a == b;
}

template <typename T>
bool verify_equality(ADLTag /*unused*/, const T &a, const T &b) {
  return custom_verify_equality(a, b);
}

template <std::integral Int, typename Rng>
Int random_element(TypeTag<Int> /*unused*/, InOut<Rng> rng) {
  std::uniform_int_distribution<Int> dist{
      std::numeric_limits<Int>::min(),
      std::numeric_limits<Int>::max()};
  return dist(*rng);
}

template <std::floating_point Float, typename Rng>
Float random_element(TypeTag<Float> /*unused*/, InOut<Rng> rng) {
  constexpr double TWO = 2.;
  std::uniform_real_distribution<Float> dist{
      -std::numeric_limits<Float>::max() / TWO,
      std::numeric_limits<Float>::max() / TWO};
  return dist(*rng);
}

bool verify_equality(ADLTag /*unused*/, double a, double b);

template <typename Rng>
google::protobuf::Timestamp random_element(
    TypeTag<google::protobuf::Timestamp> /*unused*/,
    InOut<Rng> rng) {
  google::protobuf::Timestamp result;
  result.set_seconds(random_element(TypeTag<int32_t>(), rng));
  constexpr int32_t NANOS_LB = 0;
  constexpr int32_t NANOS_UB = 1000000000;
  std::uniform_int_distribution<int32_t> dist{NANOS_LB, NANOS_UB};
  result.set_nanos(dist(*rng));
  return result;
}

bool verify_equality(
    ADLTag /*unused*/,
    const google::protobuf::Timestamp &a,
    const google::protobuf::Timestamp &b);

template <typename Rng>
time::Duration random_element(
    TypeTag<time::Duration> /*unused*/,
    InOut<Rng> rng) {
  return time::Duration{converter::random_element<int64_t>(rng)};
}

template <typename Rng>
time::Timestamp random_element(
    TypeTag<time::Timestamp> /*unused*/,
    InOut<Rng> rng) {
  return time::Timestamp{converter::random_element<time::Duration>(rng)};
}

template <typename T, typename Rng>
std::vector<T> random_element(
    TypeTag<std::vector<T>> /*unused*/,
    InOut<Rng> rng) {
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  std::vector<T> result;
  result.reserve(size);
  for (size_t ii = 0; ii < size; ++ii) {
    result.emplace_back(random_element(TypeTag<T>(), rng));
  }
  return result;
}

// Verify that two vectors of type T are equal assuming that
// verify_equality(const T &a, const T &b) is defined either in the resim
// namespace or the same namespace as T.
template <typename T>
bool verify_equality(
    ADLTag /*unused*/,
    const std::vector<T> &a,
    const std::vector<T> &b) {
  // Requires C++14 to not be undefined behavior. Thanksfully, the CTAD on the
  // righthand-side of the equality requires C++17.
  return std::mismatch(
             a.cbegin(),
             a.cend(),
             b.cbegin(),
             b.cend(),
             [](const T &a, const T &b) { return verify_equality(a, b); }) ==
         std::pair(a.cend(), b.cend());
}

template <typename T, typename Rng>
google::protobuf::RepeatedPtrField<T> random_element(
    TypeTag<google::protobuf::RepeatedPtrField<T>> /*unused*/,
    InOut<Rng> rng) {
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  google::protobuf::RepeatedPtrField<T> result;
  result.Reserve(size);
  for (size_t ii = 0; ii < size; ++ii) {
    *result.Add() = random_element(TypeTag<T>(), rng);
  }
  return result;
}

// Verify that two vectors of type T are equal assuming that
// verify_equality(const T &a, const T &b) is defined either in the resim
// namespace or the same namespace as T.
template <typename T>
bool verify_equality(
    ADLTag /*unused*/,
    const google::protobuf::RepeatedPtrField<T> &a,
    const google::protobuf::RepeatedPtrField<T> &b) {
  // Requires C++14 to not be undefined behavior. Thanksfully, the CTAD on the
  // righthand-side of the equality requires C++17.
  return std::mismatch(
             a.cbegin(),
             a.cend(),
             b.cbegin(),
             b.cend(),
             [](const T &a, const T &b) { return verify_equality(a, b); }) ==
         std::pair(a.cend(), b.cend());
}

template <typename T, typename Rng>
google::protobuf::RepeatedField<T> random_element(
    TypeTag<google::protobuf::RepeatedField<T>> /*unused*/,
    InOut<Rng> rng) {
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  google::protobuf::RepeatedField<T> result;
  result.Reserve(size);
  for (size_t ii = 0; ii < size; ++ii) {
    *result.Add() = random_element(TypeTag<T>(), rng);
  }
  return result;
}

// Verify that two vectors of type T are equal assuming that
// verify_equality(const T &a, const T &b) is defined either in the resim
// namespace or the same namespace as T.
template <typename T>
bool verify_equality(
    ADLTag /*unused*/,
    const google::protobuf::RepeatedField<T> &a,
    const google::protobuf::RepeatedField<T> &b) {
  // Requires C++14 to not be undefined behavior. Thanksfully, the CTAD on the
  // righthand-side of the equality requires C++17.
  return std::mismatch(
             a.cbegin(),
             a.cend(),
             b.cbegin(),
             b.cend(),
             [](const T &a, const T &b) { return verify_equality(a, b); }) ==
         std::pair(a.cend(), b.cend());
}

template <typename T, size_t N, typename Rng>
std::array<T, N> random_element(
    TypeTag<std::array<T, N>> /*unused*/,
    InOut<Rng> rng) {
  std::array<T, N> result;
  for (size_t ii = 0; ii < N; ++ii) {
    result.at(ii) = random_element(TypeTag<T>(), rng);
  }
  return result;
}

template <typename K, typename V, typename Rng>
std::unordered_map<K, V> random_element(
    TypeTag<std::unordered_map<K, V>> /*unused*/,
    InOut<Rng> rng) {
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  std::unordered_map<K, V> result;
  for (size_t ii = 0; ii < size; ++ii) {
    result.emplace(random_element<K>(rng), random_element<V>(rng));
  }
  return result;
}

template <typename K, typename V>
bool verify_equality(
    ADLTag /*unused*/,
    const std::unordered_map<K, V> &a,
    const std::unordered_map<K, V> &b) {
  // Requires C++14 to not be undefined behavior. Thanksfully, the CTAD on the
  // righthand-side of the equality requires C++17.
  for (const auto &[k, v] : a) {
    if (not b.contains(k) or not converter::verify_equality(v, b.at(k))) {
      return false;
    }
  }
  for (const auto &[k, v] : b) {
    if (not a.contains(k) or not converter::verify_equality(v, a.at(k))) {
      return false;
    }
  }
  return true;
}

template <typename ScalarT, int RowDim, int ColDim, typename Rng>
Eigen::Matrix<ScalarT, RowDim, ColDim> random_element(
    TypeTag<Eigen::Matrix<ScalarT, RowDim, ColDim>> /*unused*/,
    InOut<Rng> rng) {
  return testing::random_matrix<Eigen::Matrix<ScalarT, RowDim, ColDim>>(*rng);
}

template <typename ScalarT, int RowDim, int ColDim>
bool verify_equality(
    ADLTag /*unused*/,
    const Eigen::Matrix<ScalarT, RowDim, ColDim> &a,
    const Eigen::Matrix<ScalarT, RowDim, ColDim> &b) {
  return math::is_approx(a, b);
}

template <typename Rng>
std::string random_element(TypeTag<std::string> /*unused*/, InOut<Rng> rng) {
  constexpr std::array<char, 53> ALPHABET{
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"};
  constexpr size_t MIN_SIZE = 3;
  constexpr size_t MAX_SIZE = 7;
  std::uniform_int_distribution<size_t> size_dist{MIN_SIZE, MAX_SIZE};
  const size_t size = size_dist(*rng);
  std::string result(size, '\0');
  for (size_t ii = 0; ii < size; ++ii) {
    result.at(ii) = ALPHABET.at(random_element<size_t>(rng) % ALPHABET.size());
  }
  return result;
}

// POD data type auto-generation

template <Parsable T, typename Rng, size_t... Idxs>
T auto_random_element(
    TypeTag<T> tt,
    std::index_sequence<Idxs...> /*unused*/,
    InOut<Rng> rng) {
  T result;
  using Parser = decltype(get_parser(tt));
  Parser parser = get_parser(tt);
  bool unused =
      ((parser.template get<Idxs>(result) = random_element(
            TypeTag<typename Parser::template FieldType<Idxs>>(),
            rng),
        true) &&
       ...);
  (void)unused;
  return result;
}

template <Parsable T, typename Rng>
T random_element(TypeTag<T> tt, InOut<Rng> rng) {
  constexpr size_t NUM_FIELDS = decltype(get_parser(tt))::NUM_FIELDS;
  return auto_random_element(tt, std::make_index_sequence<NUM_FIELDS>(), rng);
}

template <typename T, size_t... Idxs>
bool auto_verify_equality(
    ADLTag /*unused*/,
    std::index_sequence<Idxs...> /*unused*/,
    const T &a,
    const T &b) {
  using Parser = decltype(get_parser(TypeTag<T>()));
  Parser parser = get_parser(TypeTag<T>());
  return (
      verify_equality(
          ADLTag(),
          parser.template get<Idxs>(a),
          parser.template get<Idxs>(b)) &&
      ...);
}

template <typename T>
concept CustomComparable = requires(T a, T b) {
  { custom_verify_equality(a, b) } -> std::same_as<bool>;
};

template <Parsable T>
    bool verify_equality(ADLTag /*unused*/, const T &a, const T &b) requires(
        not std::equality_comparable<T>) &&
    (not CustomComparable<T>) {
  constexpr size_t NUM_FIELDS = decltype(get_parser(
      TypeTag<std::remove_reference_t<std::remove_cv_t<T>>>()))::NUM_FIELDS;
  return auto_verify_equality(
      ADLTag(),
      std::make_index_sequence<NUM_FIELDS>(),
      a,
      b);
}

}  // namespace resim::converter
