

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

  // Proto doesn't actually add these for numeric types like int, but
  // we're just mocking this for other data types here. In practice,
  // you can't create parsers in such case since proto requires you to
  // call a setter.
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
  test_namespace::CppStruct my_struct{
      .x = 5,
      .y = 6,
  };

  // ACTION
  auto parser = get_parser(TypeTag<test_namespace::CppStruct>());

  // VERIFICATION
  EXPECT_EQ(parser.get<0>(my_struct), my_struct.x);
  EXPECT_EQ(parser.get<1>(my_struct), my_struct.y);
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
