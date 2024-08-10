#include <cstdio>
#include <string>
#include <iostream>

void foo_bar(int test1, std::string test2) {
  std::cout << test1 << " " << test2 << std::endl;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world mypkg package\n");
  return 0;
}
