# Variant Matching

## Overview

When working with variants, one very frequently encounters cases of branching
logic depending on what's present in the variant. Typically this looks like
this:

```cpp
std::variant<int, char, double, bool> my_variant = 'm';

// ...

if (std::holds_alternative<char>(my_variant)) {
  const char my_char = std::get<char>(my_variant);
  //
  // <Do something with my_char>
  //
} else if (std::holds_alternative<int>(my_variant)) {
  const int my_int = std::get<int>(my_variant);
  //
  // <Do something with my_int>
  //
} else {
  //
  // <Handle default case>
  //
}
```

It can be a bit annoying to set up this branching logic every time, and such an
approach can sometimes be clunkier when multiple variant cases can be supported
with the same logic (e.g. if we just want to convert all cases in
`std::variant<int, float, double>` to `double`). To address such cases, we
include an implementation of pattern matching that allows users to provide a
list of functors (referred to as branches) that can be applied to the current
variant to the `match()` function. The branch which best matches the current
case of the variant according to C++'s overload resolution rules will be
selected and executed on the current case of the variant. The `match()` function
then returns any value returned by the selected functor. Note that all functors
must have the same return type and all variant cases must match at least one of
the functors.

## Example

Here's how you can perform pattern matching with the `match()` function:
```cpp
std::variant<int, char, double, bool> my_variant = 'm';

// ...

std::cout
    << match(
           my_variant,
           // case: char
           [](const char c) { return "This variant contains a char!"; },
           // case: int
           [](const int i) { return "This variant contains an int!"; },
           // default:
           [](const auto x) {
             return "This variant contains a double or bool!";
           })
    << std::endl;
```


!!! danger "Gotchas / Pitfalls"
    Due to the current implementation of this functionality, users should be aware
    of a few limitations. TL;DR just wrap every branch in a non-`mutable` lambda to
    be safe if you aren't sure.
    
     * The functors passed must be *functors*. Function pointers don't
       work. As a simple work-around, users can wrap these in a
       non-`mutable` lambda.
    
     * The functors should not define non-`const` `operator()`. This admonition
       includes any lambdas marked `mutable`. Because non-`const` member functions
       are preferred over `const` in overload resolution, this can cause ambiguities
       when other branches can potentially bind the current type (e.g. a `int`
       branch binding a `char`). We considered disabling all non-const `operator()`
       overloads, but decided that this would be more likely to cause silent errors
       in most cases (i.e. a default case might simply be used when a user didn't
       expect it). If a user must use a functor with a non-`const` `operator()`,
       they can work around it again by wrapping in a non-`mutable` lambda.
    
     * Functors that are passed as lvalues will end up being copied. If a user
       wishes to avoid this as an optimization, they can do so by reference
       capturing it in a lambda that they then pass in instead.

!!! note "Development Opportunities"
     * It's not all that hard to add some functionality to convert any function
       pointers passed into `match()` into lambdas, but we haven't done it yet since
       that use case is very rare.
     
     * It's possible to avoid copying functors passed as lvalues, but we believe
       this would require a much more complex implementation than is warranted for
       this edge case.
