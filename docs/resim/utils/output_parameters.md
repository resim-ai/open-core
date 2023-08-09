# Output Parameters

## InOut Wrappers

Although the [Google C++ Style
Guide](https://google.github.io/styleguide/cppguide.html#Inputs_and_Outputs)
and consequently our style guide have a preference for return values over
output parameters, there are still cases where having a function output data
using a parameter is reasonable (e.g. when there are many outputs from a
function and making a custom struct to return them all is inconvenient).
Typically one could do this using pass-by-reference. Consider the following
simple example:

```
#include <iostream>

void set_to_three(int &x) {
  x = 3;
}

int main(int argc, char **argv) {
  int val;
  set_to_three(val);
  std::cout << val << std::endl;
  return 0;
}
```

This code as written works perfectly well. However, in practice such code can
be bugprone because developers cannot tell whether the call to `set_to_three()`
will modify its arguments. To improve code clarity, we therefore prefer the use
of our `InOut` wrapper when passing in arguments that may be modified by a
function. Using it works like so:

```
void set_to_three(resim::InOut<int> x) { 
  *x = 3; 
}

int main(int argc, char **argv) {
  int val;
  set_to_three(resim::InOut{val});
  std::cout << val << std::endl;
  return 0;
}
```

In this case, the reader immediately knows that `val` may be changed by
`set_to_three()`. Note that you can also use the arrow operator with `InOut`:

```
#include <iostream>

#include "resim/utils/inout.hh"

struct Foo {
  int x = 0;
};

void set_to_three(resim::InOut<Foo> f) { 
  f->x = 3; 
}

int main(int argc, char **argv) {
  Foo f;
  set_to_three(resim::InOut{f});
  std::cout << f.x << std::endl;
  return 0;
}
```

Under the hood, `InOut` is simply a pointer to the object passed in, so it
should be treated with care to avoid dangling references. It's only use is to
annotate the code so that users know which arguments they pass may be modified.
Typically, there is no danger of memory issues if, as in this example, we're
simply wrapping and passing a variable from the stack into our function.

## NullableReference

Sometimes we only want a function to use an output parameter conditionally.
Normally, one could use a raw pointer to do this. If the pointer is not
`nullptr`, the function populates it, but otherwise leaves it alone:

```
#include <iostream>

void maybe_set_to_three(int *x) {
  std::cout << "Ran maybe_set_to_three()!" << std::endl;
  if (x) {
    *x = 3;
  }
}

int main(int argc, char **argv) {
  int val;
  maybe_set_to_three(&val);
  std::cout << val << std::endl;
  maybe_set_to_three(nullptr);
  return 0;
}
```

As above, however, we would like to make things more explicit. In this case, we
want to make it clear that the pointer is being used as a nullable reference
and that no memory ownership is being passed from the caller to the function.
To do this, we use the `NullableReference` template:

```
#include <iostream>

#include "resim/utils/nullable_reference.hh"

void maybe_set_to_three(resim::NullableReference<int> x) {
  std::cout << "Ran maybe_set_to_three()!" << std::endl;
  if (x.has_value()) {
    *x = 3;
  }
}

int main(int argc, char **argv) {
  int val;
  maybe_set_to_three(resim::NullableReference{val});
  std::cout << val << std::endl;
  maybe_set_to_three(resim::null_reference<int>);
  return 0;
}
```

In some ways, this wrapper is almost the same as `InOut`, and one could use
`NullableReference` anywhere that an `InOut` could be used. However, it is
important to have these as separate wrappers because a function can express
more about how it treats its parameters by its choice of `InOut` (where its
clear that the function **will** output something there) or `NullableReference`
(where it's clear the function **won't** if you don't ask it to).

!!! Note
    Feel free to play around with the [source
    code](https://github.com/resim-ai/open-core/blob/main/resim/examples/output_parameters.cc)
    for the examples above.
