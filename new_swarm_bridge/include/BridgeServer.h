/*
 * @Brief: Swarm bridge for robot swarm applications based on ROS1
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-19 18:11:22 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-20 10:54:17
 */

#ifndef BRIDGE_SERVER_H
#define BRIDGE_SERVER_H

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <memory>
#include <deque>

#include <steam/steamnetworkingsockets.h>
#include <steam/isteamnetworkingsockets.h>

namespace swarm_bridge
{

/**
 * @brief 
 * 
 */
class BridgeServer
{
 public: 
  BridgeServer() = default;
  virtual ~BridgeServer() = default;
  BridgeServer(const BridgeServer &) = delete;
  BridgeServer &operator=(const BridgeServer &) = delete;
  BridgeServer(BridgeServer &&) = delete;
  BridgeServer &operator=(BridgeServer &&) = delete;

  struct DataPackage
  {
    ISteamNetworkingMessage *msg_ptr = nullptr; // this should be called with Release() when it's done
    uint8_t *data = nullptr; // decompress -> deserialize -> tuple(id, topic_name, topic_type, serialized_data) -> deserialize serialized data
    size_t size = 0;
  }; // struct DataPackage

  using DataPackageQueue_t = std::deque<DataPackage>;
  using DataPackageQueuePtr_t = std::unique_ptr<DataPackageQueue_t>;

  struct ClientWork
  {
    size_t id = 0;
    DataPackageQueuePtr_t queue_ptr = nullptr;
  }; // struct ClientWork

  void run(uint16_t port);

  std::thread spawnWorker(uint16_t port);

  std::map<HSteamNetConnection, ClientWork> &getClientMap() { return client_map_; };

  bool is_stop = false;

 private: 

  HSteamListenSocket listen_socket_handle_;
  HSteamNetPollGroup poll_group_handle_;
  ISteamNetworkingSockets *interface_ptr_;
  // (connection, client_info)
  std::map<HSteamNetConnection, ClientWork> client_map_; 

  static BridgeServer *callback_instance_ptr_; 

  void pollIncommingMessages(); 
  void pollConnectionStateChanges(); 
  void onConnectionStatusChanged(
      SteamNetConnectionStatusChangedCallback_t *info_ptr);
  static void connectionStatusChangedCallback(
      SteamNetConnectionStatusChangedCallback_t *info_ptr); 
}; // class BridgeServer

} // namespace swarm_bridge


#endif // BRIDGE_SERVER_H