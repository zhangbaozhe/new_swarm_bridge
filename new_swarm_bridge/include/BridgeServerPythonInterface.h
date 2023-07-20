/*
 * @Brief: Python interface for BridgeServer
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-19 21:10:53 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-20 10:53:08
 */

#ifndef BRIDGE_SERVER_PYTHON_INTERFACE_H
#define BRIDGE_SERVER_PYTHON_INTERFACE_H

#include "BridgeServer.h"

#include <memory>
#include <thread>
#include <mutex>
#include <vector>

struct BridgeServerPythonInterface
{
  std::unique_ptr<swarm_bridge::BridgeServer>
      bridge_server_instance_ptr = nullptr; 
  
  // call swarm_bridge::BridgeServer::run
  std::thread working_thread; 

  
  // exposed
  void init(uint16_t port); 

  // exposed
  void stop(); 

  // exposed
  void release(const std::vector<swarm_bridge::BridgeServer::DataPackage> &); 

  // exposed
  std::vector<swarm_bridge::BridgeServer::DataPackage>
  getLatestDataPackages(); 
}; // struct BridgeServerPythonInterface

#endif // BRIDGE_SERVER_PYTHON_INTERFACE