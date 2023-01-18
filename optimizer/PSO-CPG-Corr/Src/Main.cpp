#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include "PSO.h"

using namespace std;

int main(int argc, char **argv) {
  struct stat st = {0};
  if (stat("./Logs", &st) == -1) mkdir("./Logs", 0777);
  PSO pso;
  pso.initParams();
  pso.update();
  // pso.test();

  return 0;
}