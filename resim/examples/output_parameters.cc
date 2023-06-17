// This file is a companion for the Output Parameters documentation at
// https://docs.resim.ai/utils/output_parameters/

#include <iostream>

#include "resim/utils/inout.hh"
#include "resim/utils/nullable_reference.hh"

struct Foo {
  int x = 0;
};

// Forward declarations
void set_to_three(resim::InOut<int> x);
void set_to_three(resim::InOut<Foo> f);
void maybe_set_to_three(resim::NullableReference<int> x);

int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////
  // InOut Wrapper
  //////////////////////////////////////////////////////////////////////////////
  {
    int val;
    set_to_three(resim::InOut{val});
    std::cout << val << std::endl;
  }
  {
    Foo f;
    set_to_three(resim::InOut{f});
    std::cout << f.x << std::endl;
  }
  //////////////////////////////////////////////////////////////////////////////
  // NullableReference
  //////////////////////////////////////////////////////////////////////////////
  {
    int val;
    maybe_set_to_three(resim::NullableReference{val});
    std::cout << val << std::endl;
    maybe_set_to_three(resim::null_reference<int>);
  }
  return EXIT_SUCCESS;
}

// Definitions
void set_to_three(resim::InOut<int> x) { *x = 3; }

void set_to_three(resim::InOut<Foo> f) { f->x = 3; }

void maybe_set_to_three(resim::NullableReference<int> x) {
  std::cout << "Ran maybe_set_to_three()!" << std::endl;
  if (x.has_value()) {
    *x = 3;
  }
}
