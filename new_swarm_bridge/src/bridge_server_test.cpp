#include "BridgeServer.h"

int main()
{
  std::cout << "Hello" << std::endl;
  swarm_bridge::BridgeServer server;
  std::cout << "created" << std::endl;
  server.run(8888);
}