
#include "resim/converter/fuzz_helpers.hh"

#include <array>
#include <unordered_map>
#include <vector>

#include "resim/converter/fuzz_helpers_test_template.hh"
#include "resim/converter/parser.hh"
#include "resim/converter/testing/test_message.pb.h"
#include "resim/time/timestamp.hh"

namespace resim::converter {

namespace testing {

DEFINE_GET_PARSER(
    TestMessage,
    PROTO_GETTER(repeated_message_field),
    PROTO_GETTER(repeated_field))

DEFINE_GET_PARSER(TestSubMessage, PROTO_PRIMITIVE_GETTER(x))

}  // namespace testing

using Types = ::testing::Types<
    int,
    double,
    std::vector<double>,
    google::protobuf::Timestamp,
    time::Duration,
    time::Timestamp,
    std::array<double, 5>,
    std::unordered_map<std::string, int>,
    std::string,
    testing::TestMessage>;

INSTANTIATE_TYPED_TEST_SUITE_P(
    FuzzHelpersAutoTest,
    FuzzHelpersTestTemplate,
    Types);

class FuzzHelpersEdgeCasesTest : public ::testing::Test {
  static constexpr size_t SEED = 93u;

 public:
  FuzzHelpersEdgeCasesTest() : rng_{SEED} {}

  std::mt19937& rng() { return rng_; };

 private:
  std::mt19937 rng_;
};

TEST_F(FuzzHelpersEdgeCasesTest, TestMapValuesDifferent) {
  using StringToInt = std::unordered_map<std::string, int>;
  constexpr int NUM_TESTS = 10;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const StringToInt element_1{
        converter::random_element<StringToInt>(InOut{rng()})};
    StringToInt element_2{element_1};
    element_2.at(element_1.cbegin()->first) =
        converter::random_element<int>(InOut{rng()});

    StringToInt element_3{element_1};
    element_3.emplace(
        converter::random_element<std::string>(InOut{rng()}),
        converter::random_element<int>(InOut{rng()}));

    EXPECT_TRUE(converter::verify_equality(element_1, element_1));
    EXPECT_TRUE(converter::verify_equality(element_2, element_2));
    EXPECT_TRUE(converter::verify_equality(element_3, element_3));

    EXPECT_FALSE(converter::verify_equality(element_1, element_3));
    EXPECT_FALSE(converter::verify_equality(element_2, element_1));
    EXPECT_FALSE(converter::verify_equality(element_3, element_2));
    EXPECT_FALSE(converter::verify_equality(element_1, element_2));
    EXPECT_FALSE(converter::verify_equality(element_2, element_3));
    EXPECT_FALSE(converter::verify_equality(element_3, element_1));
  }
}

TEST_F(FuzzHelpersEdgeCasesTest, TestAutoGenCoverage) {
  const testing::TestMessage test_message{
      random_element<testing::TestMessage>(InOut{rng()})};

  auto test_message_different_repeated_field = test_message;
  *test_message_different_repeated_field.mutable_repeated_field() =
      random_element<google::protobuf::RepeatedField<double>>(InOut{rng()});

  auto test_message_different_repeated_message_field = test_message;
  *test_message_different_repeated_message_field
       .mutable_repeated_message_field() = random_element<
      google::protobuf::RepeatedPtrField<testing::TestSubMessage>>(
      InOut{rng()});

  EXPECT_TRUE(verify_equality(test_message, test_message));

  EXPECT_FALSE(
      verify_equality(test_message, test_message_different_repeated_field));
  EXPECT_FALSE(verify_equality(
      test_message,
      test_message_different_repeated_message_field));
  EXPECT_FALSE(
      verify_equality(test_message_different_repeated_field, test_message));
  EXPECT_FALSE(verify_equality(
      test_message_different_repeated_message_field,
      test_message));
}

TEST_F(FuzzHelpersEdgeCasesTest, TestTimestampCoverage) {
  const google::protobuf::Timestamp test_message{
      random_element<google::protobuf::Timestamp>(InOut{rng()})};

  auto test_message_different_seconds = test_message;
  test_message_different_seconds.set_seconds(
      random_element<int32_t>(InOut{rng()}));

  auto test_message_different_nanos = test_message;
  test_message_different_nanos.set_nanos(random_element<int32_t>(InOut{rng()}));

  EXPECT_TRUE(verify_equality(test_message, test_message));

  EXPECT_FALSE(verify_equality(test_message, test_message_different_seconds));
  EXPECT_FALSE(verify_equality(test_message, test_message_different_nanos));
  EXPECT_FALSE(verify_equality(test_message_different_seconds, test_message));
  EXPECT_FALSE(verify_equality(test_message_different_nanos, test_message));
}

}  // namespace resim::converter
