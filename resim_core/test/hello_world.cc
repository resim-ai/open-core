#include <iostream>

int main() {
  std::cout << "Hello, world!" << std::endl;
  std::cout << "C++: " << std::to_string(__cplusplus) << std::endl;
  std::cout << "Clang: " << __clang_major__ << "." << __clang_minor__ << "."
            << __clang_patchlevel__ << std::endl;
}
