# Error Handling

## Introduction

It is practically inevitable that errors will be encountered in any
sufficiently complex software. In order to handle errors when they occur, we
divide errors into the following categories:

 - **Recoverable Errors:** This is the class of errors where this is some
   possibility of the error being handled gracefully by the calling code. An
   example might be the case where a client is trying to connect to a server
   and it should re-try if the connection fails.

 - **Unrecoverable Errors:** This is the class of errors where there is (and
   should be) no way for the program to continue if encountered. Such an error
   means that the program is fundamentally invalid in some way. An example
   might be the case where I insert something into a `std::set` and then
   `std::map::contains()` returns false for that element. In this case, the
   contract of a specific interface has failed and there's no way for the
   caller to gracefully handle that.

Within ReSim's libraries, we have specific ways of handling each of these error
types outlined below.

## Unrecoverable Errors: REASSERT()

For unrecoverable errors we use the `REASSERT()` macro. Using this macro is very simple:

```
#include "resim/assert/assert.hh"

// ...

REASSERT(some_condition);
REASSERT(some_other_condition, "Some failure message.");
```

If `some_condition` and `some_other_condition` are both true, then this code
runs with no problem. If `some_condition` is false, the program will exit
immediately with:

```bash
terminate called after throwing an instance of 'resim::AssertException'
  what():  <path/to/source.cc:NN> - ReAssertion failed: (some_condition). Message: 
Aborted
```

If `some_other_condition` is false, then we'll get the Message field populated:

```bash
terminate called after throwing an instance of 'resim::AssertException'
  what():  <path/to/source.cc:NN> - ReAssertion failed: (some_other_condition). Message: Some failure message.
Aborted
```
Note that the text of the condition is always copied verbatim into the output. E.g.:

```
REASSERT(2 + 2 == 5);
```
Outputs:
```bash
terminate called after throwing an instance of 'resim::AssertException'
  what():  <path/to/source.cc:NN> - ReAssertion failed: (2 + 2 == 5). Message: 
Aborted
```

Under the hood, `REASSERT()` throws a `resim::AssertException` if the condition
is false. As noted below, we never catch this exception in production code, so
it always terminates the program. The reason we use an exception at all is so
that we can verify in unit tests that the assertion would terminate the program
when it should do so. We use Google Test's `EXPECT_THROW()` macro for this:

```
// my_library.cc
// ...
void my_failing_subroutine() { REASSERT(false); }

// my_library_test.cc
// ...
EXPECT_THROW(my_failing_subroutine(), AssertException);
```

## Recoverable Errors: Status & StatusValue

### Why not Exception Handling?

C++ has a built in language feature to handle this sort of error. Namely
[exception handling](https://en.cppreference.com/w/cpp/language/exceptions).
Developers can indicate that such an error has occurred by `throw`ing
exceptions which can then be caught in a `try` block and gracefully handled.
The [Google C++ Style
Guide](https://google.github.io/styleguide/cppguide.html#Exceptions) has a
detailed description of why using exceptions in this way is less than ideal. In
brief, they can lead to very non-linear control flow which makes the code
harder to reason about. Consequently, to the extent possible, we **never catch
exceptions in our production libraries**.

### The Alternative

Instead, we use function return values to pass error information up to the
caller. This is a well-trodden path with languages like
[Rust](https://doc.rust-lang.org/reference/expressions/operator-expr.html#the-question-mark-operator)
even adding language features to make this sort of error handling syntactically
nice. At ReSim we use `resim::Status` for this. For example:

```
#include "resim/assert/assert.hh"
#include "resim/utils/status.hh"

using namespace resim;

enum class Arg {
  GOOD_ARGUMENT = 0,
  BAD_ARGUMENT,
};

Status my_subroutine(const Arg arg) {
  if (arg == Arg::BAD_ARGUMENT) {
    // Use this to make a new Status object with line number information.
    return MAKE_STATUS("Oh no! We failed!");
  }
  return OKAY_STATUS;
}

// ...

Status good_status = my_subroutine(Arg::GOOD_ARGUMENT);
REASSERT(good_status.ok());
REASSERT(good_status.what() == "OKAY");

Status bad_status = my_subroutine(Arg::BAD_ARGUMENT);

REASSERT(not bad_status.ok());

// Call this macro if we decide we want to exit on bad status. I.e. if we
// don't want to gracefully handle the error, although that might be
// possible.
CHECK_STATUS_OK(bad_status);
```

When run, this code outputs the following:

```bash
terminate called after throwing an instance of 'resim::AssertException'
  what():  <path/to/source.cc:NN> - ReAssertion failed: ((bad_status).ok()). Message: {bad_status.what() == <path/to/source.cc:MM> Oh no! We failed!}
Aborted
``` 

Where `NN` is the line number where `CHECK_STATUS_OK()` is, and `MM` is the
line number where `MAKE_STATUS()` is. Note that these might not generally be in
the same source file, and we include both so that a user debugging the failure
can more easily see where things began to go wrong and also where the issue
became unrecoverable.

It's commonly the case that a function may want to handle bad statuses by
"passing the buck" up to its caller. Assuming that the enclosing function also
returns a status, we can utilize the `RETURN_IF_NOT_OK()` macro to make this
easy:

```
Status my_wrapping_subroutine(const Arg arg) {
  // This is equivalent to:
  // Status s = my_subroutine(arg);
  // if (not s.ok()) {
  //     return s;
  // }
  RETURN_IF_NOT_OK(my_subroutine(arg));
  return OKAY_STATUS;
}

// ...

Status good_status = my_wrapping_subroutine(Arg::GOOD_ARGUMENT);
REASSERT(good_status.ok());
REASSERT(good_status.what() == "OKAY");

Status bad_status = my_wrapping_subroutine(Arg::BAD_ARGUMENT);

REASSERT(not bad_status.ok());
```

### Working with Values

Sometimes, we also want to return a value from a function that can fail. To
handle this we use the `StatusValue<T>` template where `T` is the type that we
want to return. `StatusValue<T>` is designed and tested to preserve `const`ness
and `ref`ness of the contained value (i.e. you can do 
`StatusValue<const int &>` and return a wrapped reference). It also implements
the "buck-passing" behavior that `Status` does via the `RETURN_OR_ASSIGN()`
macro which works in functions returning `Status` or `StatusValue<T>` for any
`T`. Here's an example of `StatusValue<T>` in action:

```
#include "resim/utils/status.hh"
#include "resim/utils/status_value.hh"

// ...

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

// ...

Status good_status = my_outer_wrapper(Arg::GOOD_ARGUMENT);
REASSERT(good_status.ok());
REASSERT(good_status.what() == "OKAY");

Status bad_status = my_outer_wrapper(Arg::BAD_ARGUMENT);
CHECK_STATUS_OK(bad_status);
```

This code first prints `6` (for the `GOOD_ARGUMENT` case) and then outputs:

```bash
terminate called after throwing an instance of 'resim::AssertException'
  what():  <path/to/source.cc:NN> - ReAssertion failed: ((bad_status).ok()). Message: {bad_status.what() == <path/to/source.cc:MM> Oh no! We failed!}
Aborted
```

As the error propogates out of the wrappers to the `CHECK_STATUS_OK()` macro
which terminates the program.

!!! Note
    Feel free to play around with the [source
    code](https://github.com/resim-ai/open-core/blob/main/resim/examples/assert_and_status.cc)
    for the examples above.
