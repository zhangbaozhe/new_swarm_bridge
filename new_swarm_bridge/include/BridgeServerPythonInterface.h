/*
 * @Brief: Python interface for BridgeServer
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-19 21:10:53 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-21 17:18:32
 */

#ifndef BRIDGE_SERVER_PYTHON_INTERFACE_H
#define BRIDGE_SERVER_PYTHON_INTERFACE_H

#include "BridgeServer.h"

#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

struct ServerDataPackagePythonInterface
{
  swarm_bridge::BridgeServer::DataPackage data_package; 

  size_t getSize() { return data_package.size; }

  pybind11::bytes getData() { return pybind11::bytes((char *)(data_package.data), data_package.size); }

  void release() { data_package.msg_ptr->Release(); }
}; // struct ServerDataPackagePythonInterface

struct BridgeServerPythonInterface
{
  std::unique_ptr<swarm_bridge::BridgeServer>
      bridge_server_instance_ptr = nullptr; 
  
  // exposed
  void init(); 

  //exposed
  void run(int port);

  // exposed
  void stop(); 

  // exposed
  void release(std::vector<ServerDataPackagePythonInterface> &); 

  // exposed
  std::vector<ServerDataPackagePythonInterface>
  getLatestDataPackages(); 
}; // struct BridgeServerPythonInterface

#endif // BRIDGE_SERVER_PYTHON_INTERFACE