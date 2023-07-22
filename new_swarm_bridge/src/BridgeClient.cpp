#include "BridgeClient.h"

#include <cassert>

namespace swarm_bridge
{

BridgeClient *BridgeClient::callback_instance_ptr_ = nullptr;

void BridgeClient::run(const SteamNetworkingIPAddr &server_addr)
{
  interface_ptr_ = SteamNetworkingSockets(); 
  util_ptr_ = SteamNetworkingUtils();

  char szAddr[SteamNetworkingIPAddr::k_cchMaxString];
  server_addr.ToString(szAddr, sizeof(szAddr), true); 
  std::cout << "[BridgeClient::run] connecting to " << szAddr << std::endl;
  SteamNetworkingConfigValue_t opt;
  opt.SetPtr(k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged, 
      (void *)connectionStatusChangedCallback); 
  connection_handle_ = interface_ptr_->ConnectByIPAddress(server_addr, 1, &opt);

  util_ptr_->SetConnectionConfigValueInt32(connection_handle_, 
      k_ESteamNetworkingConfig_SendBufferSize, 
      1024*1024*1024); 
  
  util_ptr_->SetConnectionConfigValueInt32(connection_handle_, 
      k_ESteamNetworkingConfig_NagleTime, 
      1);

  util_ptr_->SetConnectionConfigValueInt32(connection_handle_, 
      k_ESteamNetworkingConfig_SendRateMin, 
      1024*1024); 
  util_ptr_->SetConnectionConfigValueInt32(connection_handle_, 
      k_ESteamNetworkingConfig_SendRateMax, 
      1024*1024); 

  // util_ptr_->SetGlobalConfigValueInt32(
  //     k_ESteamNetworkingConfig_FakePacketLag_Send, 
  //     100);

  if (connection_handle_ == k_HSteamNetConnection_Invalid) {
    std::cerr << "Failed to create connection" << std::endl;
    exit(1); 
  }
  work_.server_addr = server_addr;
  work_.queue_ptr = std::make_unique<DataPackageQueue_t>();
  while (!is_stop) {
    // std::cerr << "[BridgeClient::run] polling" << std::endl;
    pollIncommingMessages();
    // std::cerr << "[BridgeClient::run] polling messages" << std::endl;
    pollConnectionStateChanges(); 
    // std::cerr << "[BridgeClient::run] polling states" << std::endl;
    // std::this_thread::sleep_for(std::chrono::microseconds(10)); 
  }


}

void BridgeClient::insertWork(const DataPackage &package)
{
  work_.queue_ptr->enqueue(package); 
}

void BridgeClient::pollIncommingMessages()
{
  auto start = std::chrono::high_resolution_clock::now();
  if (! (work_.queue_ptr->size_approx() == 0)) {
    std::cerr << "[BridgeClient::pollIncommingMessages] queue size: " 
        << work_.queue_ptr->size_approx() << std::endl;
    // queue has work to do
    DataPackage temp_work; 
    work_.queue_ptr->try_dequeue(temp_work);
    // send
    auto result = interface_ptr_->SendMessageToConnection(connection_handle_, 
        (void *)temp_work.data.data(), temp_work.size, 
        k_nSteamNetworkingSend_Reliable, 
        nullptr);
    std::cerr << "[BridgeClient::pollIncommingMessages] result "
        << result << std::endl;
    // pop
    // DataPackage i;
    // work_.queue_ptr->try_dequeue(i);
    // std::cerr << "[BridgeClient::pollIncommingMessages] sent data to server "
        // << connection_handle_ << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::cerr << "[Bridgeclient::pollIncommingMessages] send used "
        << std::chrono::duration<double>(end-start).count() << std::endl;
  }
}

void BridgeClient::pollConnectionStateChanges()
{
  callback_instance_ptr_ = this;
  interface_ptr_->RunCallbacks(); 
}

void BridgeClient::onConnectionStatusChanged(
    SteamNetConnectionStatusChangedCallback_t *info_ptr)
{
  assert(info_ptr->m_hConn == connection_handle_ || connection_handle_ == k_HSteamNetConnection_Invalid);

  switch (info_ptr->m_info.m_eState)
  {
  case k_ESteamNetworkingConnectionState_None:
    break; 
  case k_ESteamNetworkingConnectionState_ClosedByPeer:
  case k_ESteamNetworkingConnectionState_ProblemDetectedLocally:
  {
    if (info_ptr->m_eOldState == k_ESteamNetworkingConnectionState_Connecting) {
      std::cerr << "[BridgeClient::onConnectionStatusChanged] "
          << "remote host lost. ("
          << info_ptr->m_info.m_szEndDebug << ")" << std::endl;
    } else if (info_ptr->m_info.m_eState == k_ESteamNetworkingConnectionState_ProblemDetectedLocally) {
      std::cerr << "[BridgeClient::onConnectionStatusChanged] "
          << "remote host lost. ("
          << info_ptr->m_info.m_szEndDebug << ")" << std::endl;
    } else {
      std::cerr << "[BridgeClient::onConnectionStatusChanged] "
          << "remote host lost. ("
          << info_ptr->m_info.m_szEndDebug << ")" << std::endl;
    }

    interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
    connection_handle_ = k_HSteamNetConnection_Invalid;
    break;
  }

  case k_ESteamNetworkingConnectionState_Connecting: 
  {
    std::cerr << "[BridgeClient::onConnectionStatusChanged] connecting" << std::endl;
    break;
  }

  case k_ESteamNetworkingConnectionState_Connected:
  {
    std::cerr << "[BridgeClient::onConnectionStatusChanged] connected" << std::endl;
    break;
  }

  default: 
    break;
  }
}

void BridgeClient::connectionStatusChangedCallback(
    SteamNetConnectionStatusChangedCallback_t *info_ptr)
{
  callback_instance_ptr_->onConnectionStatusChanged(info_ptr);
}



} // namespace swarm_bridge