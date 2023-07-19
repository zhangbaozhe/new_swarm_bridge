/*
 * @Brief: 
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-19 13:13:00 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-19 19:54:58
 */

#include <iostream>
#include <thread>
#include <chrono>

#include <steam/steamnetworkingsockets.h>
#include <steam/isteamnetworkingsockets.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/serialized_message.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

DEFINE_uint32(port, 8888, "port");

class MsgServer
{
 public: 
  void run(uint16_t port)
  {
    interface_ptr_ = SteamNetworkingSockets();

    SteamNetworkingIPAddr server_local_addr;
    server_local_addr.Clear();
    server_local_addr.m_port = port;
    SteamNetworkingConfigValue_t opt;
    opt.SetPtr(k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged, 
        (void *)steamNetConnectionStatusChangedCallback); 
    h_listen_socket_ = interface_ptr_->CreateListenSocketIP(server_local_addr, 
        1, &opt);
    if (h_listen_socket_ == k_HSteamListenSocket_Invalid) {
      std::cerr << "Failed to listen on port " << port << std::endl;
      exit(1);
    }
    h_poll_group_ = interface_ptr_->CreatePollGroup();
    if (h_poll_group_ == k_HSteamNetPollGroup_Invalid) {
      std::cerr << "Failed to listen on port " << port << std::endl;
      exit(1);
    }
    std::cout << "Server listening on port " << port << std::endl;

    while (true) {
      pollIncomingMessages();
      pollConnectionStateChanges();
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
  }
 private: 
  HSteamListenSocket h_listen_socket_;
  HSteamNetPollGroup h_poll_group_;
  ISteamNetworkingSockets *interface_ptr_;
  
  ros::NodeHandle nh_ = ros::NodeHandle();
  ros::Publisher local_str_pub1_ = 
      nh_.advertise<sensor_msgs::Imu>("/1/string_redirected", 1);

  struct ClientInfo
  {
    size_t id = 0;
  }; 

  std::map<HSteamNetConnection, ClientInfo> client_map_;

  void pollIncomingMessages() 
  {
    while (true) {
      ISteamNetworkingMessage *incomming_msg_ptr = nullptr;
      int num_msgs = interface_ptr_->ReceiveMessagesOnPollGroup(
          h_poll_group_, 
          &incomming_msg_ptr, 
          1); 
      if (num_msgs == 0)
        break;
      if (num_msgs < 0) 
        std::cerr << "Error checking for messages" << std::endl;
      assert(num_msgs == 1 && incomming_msg_ptr);
      auto client_it = client_map_.find(incomming_msg_ptr->m_conn); 
      assert(client_it != client_map_.end());
      // std::cout << "Msg received" << std::endl;
      sensor_msgs::Imu msg;
      ros::serialization::IStream stream(
          (uint8_t *)incomming_msg_ptr->m_pData, 
          incomming_msg_ptr->m_cbSize);
      ros::serialization::deserialize(stream, msg);
      local_str_pub1_.publish(msg);
    }

  }

  static MsgServer *callback_instance_ptr_;
  void onSteamNetConnectionStatusChanged(SteamNetConnectionStatusChangedCallback_t *info_ptr)
  {
    char temp[1024];
    switch (info_ptr->m_info.m_eState) {
      case k_ESteamNetworkingConnectionState_None: 
        break;
      case k_ESteamNetworkingConnectionState_ClosedByPeer: 
      case k_ESteamNetworkingConnectionState_ProblemDetectedLocally: 
      {
        if (info_ptr->m_eOldState == k_ESteamNetworkingConnectionState_Connected) {
          auto client_it = client_map_.find(info_ptr->m_hConn);
          assert(client_it != client_map_.end());

          const char *psz_debug_log_action;
          if (info_ptr->m_info.m_eState == k_ESteamNetworkingConnectionState_ProblemDetectedLocally) {
            psz_debug_log_action = "problem detected locally";
            sprintf(temp, "%d shutting down. %s", client_it->second.id, info_ptr->m_info.m_szEndDebug);
          } else {
            psz_debug_log_action = "closed by peer";
            sprintf(temp, "%d closed", client_it->second.id); 
          }

          printf( "Connection %s %s, reason %d: %s\n",
						info_ptr->m_info.m_szConnectionDescription,
						psz_debug_log_action,
						info_ptr->m_info.m_eEndReason,
						info_ptr->m_info.m_szEndDebug
					);

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
        printf("Connection request from %s", 
            info_ptr->m_info.m_szConnectionDescription);
        if (interface_ptr_->AcceptConnection(info_ptr->m_hConn) != k_EResultOK) {
          interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
          printf("Cannot accept connection");
          break;
        }

        if (!interface_ptr_->SetConnectionPollGroup(info_ptr->m_hConn, h_poll_group_)) {
          interface_ptr_->CloseConnection(info_ptr->m_hConn, 0, nullptr, false);
          printf("Failed to set poll group");
          break;
        }

        size_t id = client_map_.size() + 1;
        ClientInfo client; client.id = id;
        client_map_[info_ptr->m_hConn] = client;
        break;
      }

      case k_ESteamNetworkingConnectionState_Connected: 
        break;
      default: 
        break;
    }
  }

  static void steamNetConnectionStatusChangedCallback(SteamNetConnectionStatusChangedCallback_t *info_ptr)
  {
    callback_instance_ptr_->onSteamNetConnectionStatusChanged(info_ptr);
  }

  void pollConnectionStateChanges()
  {
    callback_instance_ptr_ = this;
    interface_ptr_->RunCallbacks();
  }
}; 

MsgServer *MsgServer::callback_instance_ptr_ = nullptr;

static void InitSteamDatagramConnectionSockets()
{
	#ifdef STEAMNETWORKINGSOCKETS_OPENSOURCE
		SteamDatagramErrMsg errMsg;
		if ( !GameNetworkingSockets_Init( nullptr, errMsg ) )
			printf( "GameNetworkingSockets_Init failed.  %s", errMsg );
	#else
		SteamDatagram_SetAppID( 570 ); // Just set something, doesn't matter what
		SteamDatagram_SetUniverse( false, k_EUniverseDev );

		SteamDatagramErrMsg errMsg;
		if ( !SteamDatagramClient_Init( errMsg ) )
			FatalError( "SteamDatagramClient_Init failed.  %s", errMsg );

		// Disable authentication when running with Steam, for this
		// example, since we're not a real app.
		//
		// Authentication is disabled automatically in the open-source
		// version since we don't have a trusted third party to issue
		// certs.
		SteamNetworkingUtils()->SetGlobalConfigValueInt32( k_ESteamNetworkingConfig_IP_AllowWithoutAuth, 1 );
	#endif

}

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "server");

  InitSteamDatagramConnectionSockets(); 
  
  MsgServer server;
  server.run(FLAGS_port);

  return 0;
}