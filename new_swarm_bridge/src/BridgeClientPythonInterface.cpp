#include "BridgeClientPythonInterface.h"

namespace py = pybind11;

void BridgeClientPythonInterface::init()
{
  bridge_client_instance_ptr = std::make_unique<
      swarm_bridge::BridgeClient>();
  py::print("[BridgeClientPythonInterface::init] unique pointer established");
}

void BridgeClientPythonInterface::run(const std::string &ip_addr, int port)
{
  SteamNetworkingIPAddr server_addr;
  server_addr.Clear();
  if (!server_addr.ParseString(ip_addr.c_str()) ){
    std::cerr << "Wrong ip format" << std::endl;
    exit(1);
  }
  server_addr.m_port = (uint16_t)port;
  py::gil_scoped_release release;
  bridge_client_instance_ptr->run(server_addr);
  py::gil_scoped_acquire acquire;
}

void BridgeClientPythonInterface::stop() 
{
  bridge_client_instance_ptr->is_stop = true;
}

void BridgeClientPythonInterface::setDataPackages(
    const std::vector<ClientDataPackagePythonInterface> &packages)
{
  for (const auto &i : packages) {
    // std::cerr << "[BridgeClientPythonInterface::setDataPackages] package data: " 
    //     << (char *)i.data_package.data.data()
    //     << std::endl; 
    bridge_client_instance_ptr->insertWork(i.data_package);
  }
}

