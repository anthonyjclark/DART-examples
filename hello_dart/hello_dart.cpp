#include <iostream>
#include <dart/dart.hpp>

using namespace dart;

int main()
{
  auto skel = dynamics::Skeleton::create();

  std::cout << "Skeleton [" << skel->getName() << "] has ["
            << skel->getNumBodyNodes() << "] bodies.\n";

  return 0;
}
