#include "BridgeServer.h"

#include <cassert>

#include <pybind11/pybind11.h>

namespace py = pybind11;


namespace swarm_bridge
{

void BridgeServer::run(uint16_t port)
{
  // py::print("[BridgeServer::run] start");
  interface_ptr_ = SteamNetworkingSockets(); 
  // py::print("[BridgeServer::run] sockets created");
  SteamNetworkingIPAddr server_local_addr;
  server_local_addr.Clear(); 
  server_local_addr.m_port = port;
  SteamNetworkingConfigValue_t opt; 
  opt.SetPtr(k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged, 
      (void *)connectionStatusChangedCallback); 
  // py::print("[BridgeServer::run] callback set");
  listen_socket_handle_ = interface_ptr_->CreateListenSocketIP(server_local_addr, 
      1, &opt); 
  // py::print("[BridgeServer::run] listen handle created");
  if (listen_socket_handle_ == k_HSteamListenSocket_Invalid) {
    std::cerr << "[BridgeServer::run] Failed to listen on port " << port << std::endl;
    is_stop = true; 
    exit(1);
  }
  poll_group_handle_ = interface_ptr_->CreatePollGroup(); 
  if (poll_group_handle_ == k_HSteamNetPollGroup_Invalid) {
    std::cerr << "[BridgeServer::run] Failed to create poll group" << std::endl;
    is_stop = true; 
    exit(1);
  }

  std::cout << "[BridgeServer::run] Server starts listening on port " << port << std::endl;
  // py::print("[BridgeServer::run] start listening");

  while (!is_stop) {
    pollIncommingMessages(); 
    pollConnectionStateChanges(); 
    // sleep for a while ~10us
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

std::thread BridgeServer::spawnWorker(uint16_t port)
{
  py::print("[BridgeServer::spawnWorker] ready to spawn");
  return std::thread([this, port]{ run(port); }); 
}

void BridgeServer::pollIncommingMessages()
{
  while (!is_stop) {
    ISteamNetworkingMessage *incomming_msg_ptr = nullptr; 
    int num_msgs = interface_ptr_->ReceiveMessagesOnPollGroup(
        poll_group_handle_, 
        &incomming_msg_ptr, 
        1); 
    if (num_msgs == 0) {
      std::cout << "[BridgeServer::pollIncommingMessages] No message" << std::endl;
      break;
    }
    if (num_msgs < 0) {
      std::cerr << "[BridgeServer::pollIncommingMessages] Error checking for messages" << std::endl; 
    }
    assert(num_msgs == 1 && incomming_msg_ptr);
    auto client_it = client_map_.find(incomming_msg_ptr->m_conn);
    assert(client_it != client_map_.end());

    DataPackage msg;
    msg.msg_ptr = incomming_msg_ptr; // (zhangbaozhe)TODO: remember to Release() 
    msg.data = static_cast<uint8_t *>(incomming_msg_ptr->m_pData); 
    msg.size = incomming_msg_ptr->m_cbSize; 
    client_it->second.queue_ptr->emplace_back(std::move(msg));

  }
}

void BridgeServer::onConnectionStatusChanged(
    SteamNetConnectionStatusChangedCallback_t *info_ptr)
{
  std::string logging_message;
  switch (info_ptr->m_info.m_eState)
  {
  case k_ESteamNetworkingConnectionState_None:
    break;

  case k_ESteamNetworkingConnectionState_ClosedByPeer: 
  case k_ESteamNetworkingConnectionState_ProblemDetectedLocally: 
  {
    // connection failed
    if (info_ptr->m_eOldState == k_ESteamNetworkingConnectionState_Connected) {
      auto client_it = client_map_.find(info_ptr->m_hConn);
      assert(client_it != client_map_.end()); 

      const char *debug_log_action; 
      if (info_ptr->m_info.m_eState == k_ESteamNetworkingConnectionState_ProblemDetectedLocally) {
        debug_log_action = "problem detected locally. "; 
        logging_message += "Robot: " 
            + std::to_string(client_it->second.id)
            + " shutting down. "
            + info_ptr->m_info.m_szEndDebug; 
      } else {
        debug_log_action = "closed by peer"; 
        logging_message += "Robot: "
            + std::to_string(client_it->second.id) 
            + " closed."; 
      }

      std::cout << "[BridgeServer::onConnectionStatusChanged] "
          << "Connection "
          << info_ptr->m_info.m_szConnectionDescription << " "
          << debug_log_action << ", "
          << "reason "
          << info_ptr->m_info.m_eEndReason << ": "
          << info_ptr->m_info.m_szEndDebug << std::endl;
      
      client_map_.erase(client_it); 
    } else {
      assert(info_ptr->m_eOldState == k_ESteamNetworkingConnectionState_Connecting); 
    }

    interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
    break;
  }

  case k_ESteamNetworkingConnectionState_Connecting: 
  {
    assert(client_map_.find(info_ptr->m_hConn) == client_map_.end());
    std::cout << "[BridgeServer::onConnectionStatusChanged] Connection request from "
        << info_ptr->m_info.m_szConnectionDescription; 
    if (interface_ptr_->AcceptConnection(info_ptr->m_hConn) != k_EResultOK) {
      interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
      std::cout << " Cannot accept connection" << std::endl;
      break;
    }

    if (!interface_ptr_->SetConnectionPollGroup(info_ptr->m_hConn, poll_group_handle_)) {
      interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
      std::cout << " Failed to set poll group" << std::endl;
      break;
    }

    ClientWork client_work; 
    client_work.id = SIZE_MAX; // to be set when the package arrives
    client_work.queue_ptr = std::make_unique<DataPackageQueue_t>();
    client_map_[info_ptr->m_hConn] = std::move(client_work); 
    std::cout << " Connected" << std::endl;
    break; 
  }

  case k_ESteamNetworkingConnectionState_Connected: 
    break;
  default:
    break;
  }
}

BridgeServer *BridgeServer::callback_instance_ptr_ = nullptr; 

void BridgeServer::connectionStatusChangedCallback(
    SteamNetConnectionStatusChangedCallback_t *info_ptr)
{
  callback_instance_ptr_->onConnectionStatusChanged(info_ptr); 
}

void BridgeServer::pollConnectionStateChanges()
{
  callback_instance_ptr_ = this;
  interface_ptr_->RunCallbacks();
}
} // namespace swarm_bridge