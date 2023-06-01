#include "resim_core/transforms/frame.hh"

#include <gtest/gtest.h>

namespace resim::transforms {

TEST(FramedTest, ConstructionAndEquality) {
  const Frame<3> A = Frame<3>::new_frame();
  const Frame<3> B = Frame<3>::new_frame();
  const Frame<3> C(A.id());  // construct with the same id as A.
  EXPECT_NE(A, B);
  EXPECT_EQ(A, C);
}

TEST(FramedTest, CopyConstructionAndAssignment) {
  const Frame<3> A = Frame<3>::new_frame();
  const Frame<3> B = Frame<3>::new_frame();
  const Frame<3> C(A);
  const Frame<3> D = B;
  EXPECT_NE(A, B);
  EXPECT_EQ(A, C);
  EXPECT_EQ(B, D);
  EXPECT_NE(C, D);
}

TEST(FramedTest, NullFrames) {
  const Frame<3> A = Frame<3>();
  const Frame<3> B = Frame<3>();
  const Frame<3> C = Frame<3>::new_frame();
  EXPECT_EQ(A, B);
  EXPECT_NE(A, C);
  EXPECT_TRUE(A.is_null());
  EXPECT_TRUE(B.is_null());
  EXPECT_FALSE(C.is_null());
}

}  // namespace resim::transforms
