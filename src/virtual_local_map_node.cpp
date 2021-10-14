#include "virtual_local_map.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "virtual_local_map");
  VirtualLocalMap node;
  node.run();
  return 0;
}



