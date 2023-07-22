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
    if (i.second.queue_ptr->size_approx() == 0)
      continue;
    swarm_bridge::BridgeServer::DataPackage package;
    i.second.queue_ptr->try_dequeue(package);     
    result.emplace_back(ServerDataPackagePythonInterface{package}); 
    std::cerr << "[BridgeServerPythonInterface::getLatestDataPackage] queue size: "
        << i.second.queue_ptr->size_approx();
  }
  return result; 
}


