
#include "resim/utils/tuple_utils.hh"

#include <gtest/gtest.h>

namespace resim {

TEST(TupleUtilsTest, TestForEachInTuple) {
  // SETUP
  // NOLINTBEGIN(readability-magic-numbers)
  auto simple_tuple = std::make_tuple(1, 2., 3U);
  // NOLINTEND(readability-magic-numbers)
  const auto add_one = [](auto number) { return number + 1; };

  // ACTION
  auto result = for_each_in_tuple(add_one, simple_tuple);

  // VERIFICATION
  static_assert(
      std::tuple_size_v<decltype(simple_tuple)> ==
      std::tuple_size_v<decltype(result)>);

  EXPECT_EQ(std::get<0>(simple_tuple) + 1, std::get<0>(result));
  EXPECT_EQ(std::get<1>(simple_tuple) + 1, std::get<1>(result));
  EXPECT_EQ(std::get<2>(simple_tuple) + 1, std::get<2>(result));
}

TEST(TupleUtilsTest, TestForEachInTupleReferences) {
  // SETUP
  int a = 1;
  int b = 2;
  int c = 3;

  // We can't copy tuples containing rvalue references, so we have to manually
  // re-construct.
  using TestTupleType =
      std::tuple<int, int &, const int &, int &&, const int &&>;
  // NOLINTBEGIN(readability-magic-numbers)
  TestTupleType test_tuple{a, b, c, 4, 5};
  TestTupleType test_tuple_to_move{a, b, c, 4, 5};
  const TestTupleType test_tuple_const{a, b, c, 4, 5};
  const TestTupleType test_tuple_const_to_move{a, b, c, 4, 5};
  // NOLINTEND(readability-magic-numbers)
  //
  const auto identity = [](auto &&x) -> decltype(x) {
    return std::forward<decltype(x)>(x);
  };

  // ACTION
  auto result = for_each_in_tuple(identity, test_tuple);
  auto result_moved =
      for_each_in_tuple(identity, std::move(test_tuple_to_move));
  auto result_from_const = for_each_in_tuple(identity, test_tuple_const);

  // This one is a bit weird and you wouldn't really ever do this, but we'll
  // test it anyway.
  auto result_moved_from_const = for_each_in_tuple(
      identity,
      static_cast<const TestTupleType &&>(test_tuple_const_to_move));

  // VERIFICATION

  // Testing reference collapsing rules. The references from the input tuple are
  // added to the references of the entry. Since result was made from an lvalue
  // bound by lvalue reference, all rvalues become lvalues.
  static_assert(std::is_same_v<
                decltype(result),
                std::tuple<int &, int &, const int &, int &, const int &>>);

  // Since result was made from an rvalue bound by rvalue refernece, all lvalues
  // stay lvalues and all rvalues stay rvalues.
  static_assert(std::is_same_v<
                decltype(result_moved),
                std::tuple<int &&, int &, const int &, int &&, const int &&>>);

  // Here, we need to be aware of how const interacts with reference collapsing.
  // The suprising cases are that you can't add const to references,
  // but you can to normal types. Therefore, collapsing const & from the tuple
  // with int & or int && leaves these types non-const while mapping int to
  // const int &.
  static_assert(
      std::is_same_v<
          decltype(result_from_const),
          std::tuple<const int &, int &, const int &, int &, const int &>>);

  static_assert(
      std::is_same_v<
          decltype(result_moved_from_const),
          std::tuple<const int &&, int &, const int &, int &&, const int &&>>);

  EXPECT_EQ(&std::get<0>(test_tuple), &std::get<0>(result));
  EXPECT_EQ(&std::get<1>(test_tuple), &std::get<1>(result));
  EXPECT_EQ(&std::get<2>(test_tuple), &std::get<2>(result));
  EXPECT_EQ(&std::get<3>(test_tuple), &std::get<3>(result));
  EXPECT_EQ(&std::get<4>(test_tuple), &std::get<4>(result));

  EXPECT_EQ(&std::get<0>(test_tuple_to_move), &std::get<0>(result_moved));
  EXPECT_EQ(&std::get<1>(test_tuple_to_move), &std::get<1>(result_moved));
  EXPECT_EQ(&std::get<2>(test_tuple_to_move), &std::get<2>(result_moved));
  EXPECT_EQ(&std::get<3>(test_tuple_to_move), &std::get<3>(result_moved));
  EXPECT_EQ(&std::get<4>(test_tuple_to_move), &std::get<4>(result_moved));

  EXPECT_EQ(&std::get<0>(test_tuple_const), &std::get<0>(result_from_const));
  EXPECT_EQ(&std::get<1>(test_tuple_const), &std::get<1>(result_from_const));
  EXPECT_EQ(&std::get<2>(test_tuple_const), &std::get<2>(result_from_const));
  EXPECT_EQ(&std::get<3>(test_tuple_const), &std::get<3>(result_from_const));
  EXPECT_EQ(&std::get<4>(test_tuple_const), &std::get<4>(result_from_const));

  EXPECT_EQ(
      &std::get<0>(test_tuple_const_to_move),
      &std::get<0>(result_moved_from_const));
  EXPECT_EQ(
      &std::get<1>(test_tuple_const_to_move),
      &std::get<1>(result_moved_from_const));
  EXPECT_EQ(
      &std::get<2>(test_tuple_const_to_move),
      &std::get<2>(result_moved_from_const));
  EXPECT_EQ(
      &std::get<3>(test_tuple_const_to_move),
      &std::get<3>(result_moved_from_const));
  EXPECT_EQ(
      &std::get<4>(test_tuple_const_to_move),
      &std::get<4>(result_moved_from_const));
}

};  // namespace resim
