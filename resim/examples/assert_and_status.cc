// This file is a companion for the Error handling documentation at
// https://docs.resim.ai/assert/

#include <cstdlib>
#include <iostream>

#include "resim/assert/assert.hh"
#include "resim/utils/status.hh"
#include "resim/utils/status_value.hh"

using resim::OKAY_STATUS;
using resim::Status;

template <typename T>
using StatusValue = resim::StatusValue<T>;

// Forward declarations of types and functions used to demonstrate Status and
// StatusValue
enum class Arg {
  GOOD_ARGUMENT = 0,
  BAD_ARGUMENT,
};

// This function returns a bad status if BAD_ARGUMENT is passed and an okay
// status if GOOD_ARGUMENT is passed.
Status my_subroutine(Arg arg);

// This function calls my_subroutine() with the given arg and uses the
// RETURN_IF_NOT_OK() macro to return a bad status if my_subroutine() does.
Status my_wrapping_subroutine(Arg arg);

// This function returns the value 3 if GOOD_ARGUMENT is passed and a bad
// status if BAD_ARGUMENT is passed.
StatusValue<int> my_returning_subroutine(Arg arg);

// This function calls my_returning_subroutine() with the given arg and uses
// RETURN_OR_ASSIGN() to return a bad status if my_returning_subroutine() does.
// Otherwise, it returns twice the value that my_returning_subroutine()
// returns (as a double this time).
StatusValue<double> my_returning_wrapper(Arg arg);

// This function wraps my_returning_wrapper() (which means it double wraps
// my_returning_subroutine()) and returns a bad status if it does.
// Alternatively, it prints the result given by my_returning_wrapper() if it is
// successful.
Status my_outer_wrapper(Arg arg);

int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////
  // Basic REASSERT usage
  //////////////////////////////////////////////////////////////////////////////
  {
    const bool some_condition = true;
    const bool some_other_condition = true;
    REASSERT(some_condition);
    REASSERT(some_other_condition, "Some failure message.");

    // REASSERT(2 + 2 == 5);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Status usage
  //////////////////////////////////////////////////////////////////////////////
  {
    Status good_status = my_subroutine(Arg::GOOD_ARGUMENT);
    REASSERT(good_status.ok());
    REASSERT(good_status.what() == "OKAY");

    Status bad_status = my_subroutine(Arg::BAD_ARGUMENT);

    REASSERT(not bad_status.ok());

    // Call this macro if we decide we want to exit on bad status. I.e. if we
    // don't want to gracefully handle the error, although that might be
    // possible.
    // CHECK_STATUS_OK(bad_status);
  }

  //////////////////////////////////////////////////////////////////////////////
  // RETURN_IF_NOT_OK usage
  //////////////////////////////////////////////////////////////////////////////
  {
    Status good_status = my_wrapping_subroutine(Arg::GOOD_ARGUMENT);
    REASSERT(good_status.ok());
    REASSERT(good_status.what() == "OKAY");

    Status bad_status = my_wrapping_subroutine(Arg::BAD_ARGUMENT);

    REASSERT(not bad_status.ok());
  }

  //////////////////////////////////////////////////////////////////////////////
  // StatusValue usage
  //////////////////////////////////////////////////////////////////////////////
  {
    Status good_status = my_outer_wrapper(Arg::GOOD_ARGUMENT);
    REASSERT(good_status.ok());
    REASSERT(good_status.what() == "OKAY");

    Status bad_status = my_outer_wrapper(Arg::BAD_ARGUMENT);
    // CHECK_STATUS_OK(bad_status);
  }

  return EXIT_SUCCESS;
}

Status my_subroutine(const Arg arg) {
  if (arg == Arg::BAD_ARGUMENT) {
    // Use this to make a new Status object with line number information.
    return MAKE_STATUS("Oh no! We failed!");
  }
  return OKAY_STATUS;
}

Status my_wrapping_subroutine(const Arg arg) {
  // This is equivalent to:
  // Status s = my_subroutine(arg);
  // if (not s.ok()) {
  //     return s;
  // }
  RETURN_IF_NOT_OK(my_subroutine(arg));
  return OKAY_STATUS;
}

StatusValue<int> my_returning_subroutine(const Arg arg) {
  if (arg == Arg::BAD_ARGUMENT) {
    return MAKE_STATUS("Oh no! We failed!");
  }
  return 3;
}

StatusValue<double> my_returning_wrapper(const Arg arg) {
  int val = RETURN_OR_ASSIGN(my_returning_subroutine(arg));
  return 2.0 * val;
}

Status my_outer_wrapper(const Arg arg) {
  double val = RETURN_OR_ASSIGN(my_returning_wrapper(arg));
  std::cout << val << std::endl;
  return OKAY_STATUS;
}
