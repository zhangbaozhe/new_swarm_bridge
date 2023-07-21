#include "BridgeServerPythonInterface.h"


namespace py = pybind11; 

void BridgeServerPythonInterface::init()
{
  bridge_server_instance_ptr = std::make_unique<
      swarm_bridge::BridgeServer>();
  py::print("[BridgeServerPythonInterface::init] unique pointer established");
}

void BridgeServerPythonInterface::run(int port)
{
  py::gil_scoped_release release; 
  bridge_server_instance_ptr->run(port);
  py::gil_scoped_acquire acquire;
}

void BridgeServerPythonInterface::stop()
{
  bridge_server_instance_ptr->is_stop = true;
}

void BridgeServerPythonInterface::release(
    std::vector<ServerDataPackagePythonInterface> &data_packages)
{
  for (auto &package : data_packages) {
    package.release(); 
  }
}

std::vector<ServerDataPackagePythonInterface>
BridgeServerPythonInterface::getLatestDataPackages()
{
  std::vector<ServerDataPackagePythonInterface> result;
  auto &client_map = bridge_server_instance_ptr->getClientMap(); 
  for (const auto &i : client_map) {
    if (i.second.queue_ptr->empty())
      continue;
    result.emplace_back(ServerDataPackagePythonInterface{i.second.queue_ptr->front()}); 
    i.second.queue_ptr->pop_front(); 
  }
  return result; 
}


