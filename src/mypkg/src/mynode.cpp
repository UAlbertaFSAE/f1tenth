#include <cstdio>
#include <iostream>
#include <string>

void foo_bar(int test1, std::string& test2) {
  std::cout << test1 << " " << test2 << '\n';
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  std::cout << "hello world mypkg package\n";
  return 0;
}
