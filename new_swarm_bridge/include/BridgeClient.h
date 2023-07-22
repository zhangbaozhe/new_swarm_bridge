/*
 * @Brief: 
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-21 12:33:29 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-21 16:29:16
 */

#ifndef BRIDGE_CLIENT_H
#define BRIDGE_CLIENT_H

#include "ReadWriteQueue.hpp"

#include <iostream>
#include <thread>
#include <chrono>
// #include <deque>
#include <vector>
#include <map>
#include <memory>

#include <steam/steamnetworkingsockets.h>
#include <steam/isteamnetworkingsockets.h>
#include <steam/isteamnetworkingutils.h>

namespace swarm_bridge
{

/**
 * @brief One client deals with one server (TBD)
 * 
 */
class BridgeClient
{
 public:
  BridgeClient()
  {
    SteamDatagramErrMsg errMsg;
		if ( !GameNetworkingSockets_Init( nullptr, errMsg ) )
			printf( "GameNetworkingSockets_Init failed.  %s\n", errMsg );
  }
  virtual ~BridgeClient() = default; 
  BridgeClient(const BridgeClient &) = delete; 
  BridgeClient &operator=(const BridgeClient &) = delete; 
  BridgeClient(BridgeClient &&) = delete;
  BridgeClient &operator=(BridgeClient &&) = delete; 

  struct DataPackage
  {
    std::vector<uint8_t> data;
    size_t size = 0;
  }; // struct DataPackage

  using DataPackageQueue_t = moodycamel::ReaderWriterQueue<DataPackage>;
  using DataPackageQueuePtr_t = std::unique_ptr<DataPackageQueue_t>; 

  struct ServerWork
  {
    SteamNetworkingIPAddr server_addr; 
    DataPackageQueuePtr_t queue_ptr = nullptr; 
  }; // struct ServerWork


  /**
   * @brief Connects to one server
   * 
   * @param server_addr 
   */
  void run(const SteamNetworkingIPAddr &server_addr); 

  /**
   * @brief Insert data to the corresponding queue
   * 
   * @param server_addr 
   * @param data 
   */
  void insertWork(const DataPackage &package); 
  
  std::atomic_bool is_stop{false};
  
 private: 
  ISteamNetworkingSockets *interface_ptr_ = nullptr; 
  ISteamNetworkingUtils *util_ptr_ = nullptr;
  HSteamNetConnection connection_handle_;
  ServerWork work_;

  static BridgeClient *callback_instance_ptr_;

  void pollIncommingMessages();
  void pollConnectionStateChanges(); 
  void onConnectionStatusChanged(
      SteamNetConnectionStatusChangedCallback_t *info_ptr); 
  static void connectionStatusChangedCallback(
      SteamNetConnectionStatusChangedCallback_t *info_ptr); 
}; // class BridgeClient
}

#endif // BRIDGE_CLIENT_H