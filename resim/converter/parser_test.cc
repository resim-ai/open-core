// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/converter/parser.hh"

#include <gtest/gtest.h>

#include <utility>

#include "resim/converter/tags.hh"

namespace resim::converter {

namespace test_namespace {

struct CppStruct {
  int x = 0;
  int y = 0;
};

class MockProto {
 public:
  MockProto(int x, std::string y) : x_{x}, y_{std::move(y)} {}

  int x() const { return x_; }
  const std::string &y() const { return y_; }

  void set_x(const int val) { x_ = val; }
  std::string *mutable_y() { return &y_; }

 private:
  int x_ = 0;
  std::string y_;
};

DEFINE_GET_PARSER(CppStruct, POD_GETTER(x), POD_GETTER(y));
DEFINE_GET_PARSER(MockProto, PROTO_PRIMITIVE_GETTER(x), PROTO_GETTER(y));

}  // namespace test_namespace

TEST(ParserTest, TestParserPOD) {
  // SETUP
  constexpr int X_VAL = 5;
  constexpr int Y_VAL = 6;
  constexpr int NEW_X_VAL = 5;
  constexpr int NEW_Y_VAL = 6;

  const test_namespace::CppStruct my_struct{
      .x = X_VAL,
      .y = Y_VAL,
  };

  test_namespace::CppStruct my_mutable_struct{
      .x = X_VAL,
      .y = Y_VAL,
  };

  // ACTION
  auto parser = get_parser(TypeTag<test_namespace::CppStruct>());
  parser.get<0>(my_mutable_struct) = NEW_X_VAL;
  parser.get<1>(my_mutable_struct) = NEW_Y_VAL;

  // VERIFICATION
  EXPECT_EQ(&parser.get<0>(my_struct), &my_struct.x);
  EXPECT_EQ(&parser.get<1>(my_struct), &my_struct.y);
  EXPECT_EQ(parser.get<0>(my_mutable_struct), NEW_X_VAL);
  EXPECT_EQ(parser.get<1>(my_mutable_struct), NEW_Y_VAL);
}

TEST(ParserTest, TestParserProto) {
  // SETUP
  constexpr int X = 5;
  test_namespace::MockProto mock_proto{X, "my_y_value"};
  const auto mock_proto_const = mock_proto;

  // ACTION
  auto parser = get_parser(TypeTag<test_namespace::MockProto>());

  // VERIFICATION
  EXPECT_EQ(parser.get<0>(mock_proto_const), mock_proto.x());
  EXPECT_EQ(parser.get<1>(mock_proto_const), mock_proto.y());

  EXPECT_EQ(parser.get<0>(mock_proto), mock_proto.x());
  EXPECT_EQ(parser.get<1>(mock_proto), mock_proto.y());
}

}  // namespace resim::converter
