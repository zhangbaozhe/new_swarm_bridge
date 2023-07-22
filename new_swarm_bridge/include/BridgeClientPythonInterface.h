/*
 * @Brief: 
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-21 14:45:43 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-21 16:31:49
 */

#ifndef BRIDGE_CLIENT_PYTHON_INTERFACE_H
#define BRIDGE_CLIENT_PYTHON_INTERFACE_H

#include "BridgeClient.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

struct ClientDataPackagePythonInterface
{
  swarm_bridge::BridgeClient::DataPackage data_package;

  void setSize(int size) { data_package.size = (size_t)size; }

  void setData(char *data) 
  { 
    // std::cerr << "[ClientDataPackagePythonInterface::setData] data: " << (size_t)data << std::endl;
    // data_package.data = (uint8_t *)(data); 
    data_package.data.resize(data_package.size);
    // TODO: may have performance issue
    std::memcpy(data_package.data.data(), data, data_package.size);
    // std::cerr << "[ClientDataPackagePythonInterface::setData] data: " << (size_t)data_package.data.data() << std::endl;
  }

}; // struct ClientDataPackagePythonInterface

struct BridgeClientPythonInterface
{
  std::unique_ptr<swarm_bridge::BridgeClient>
      bridge_client_instance_ptr = nullptr;
  
  void init();

  void run(const std::string &ip_addr, int port); 

  void stop(); 

  void setDataPackages(const std::vector<ClientDataPackagePythonInterface> &); 

}; // struct BridgeClientPythonInterface


#endif // BRIDGE_CLIENT_PYTHON_INTERFACE_H