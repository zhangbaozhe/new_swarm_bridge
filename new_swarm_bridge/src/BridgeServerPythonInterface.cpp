#include "BridgeServerPythonInterface.h"

#include <pybind11/pybind11.h>

void BridgeServerPythonInterface::init(uint16_t port)
{
  bridge_server_instance_ptr = std::make_unique<
      swarm_bridge::BridgeServer>();
  working_thread = bridge_server_instance_ptr->spawnWorker(port); 
}

void BridgeServerPythonInterface::stop()
{
  bridge_server_instance_ptr->is_stop = true;
  if (working_thread.joinable()) 
    working_thread.join(); 
}

void BridgeServerPythonInterface::release(
    const std::vector<swarm_bridge::BridgeServer::DataPackage> &data_packages)
{
  for (auto &package : data_packages) {
    package.msg_ptr->Release(); 
  }
}

std::vector<swarm_bridge::BridgeServer::DataPackage>
BridgeServerPythonInterface::getLatestDataPackages()
{
  std::vector<swarm_bridge::BridgeServer::DataPackage> result;
  auto &client_map = bridge_server_instance_ptr->getClientMap(); 
  for (const auto &i : client_map) {
    result.push_back(i.second.queue_ptr->front()); 
    i.second.queue_ptr->pop_front(); 
  }
  return result; 
}

namespace py = pybind11; 

PYBIND11_MODULE(Bridge, m) {
  py::class_<BridgeServerPythonInterface>(m, "BridgeServer")
      .def("init", &BridgeServerPythonInterface::init)
      .def("stop", &BridgeServerPythonInterface::stop)
      .def("release", &BridgeServerPythonInterface::release)
      .def("getLatestDataPackages", &BridgeServerPythonInterface::getLatestDataPackages);
}