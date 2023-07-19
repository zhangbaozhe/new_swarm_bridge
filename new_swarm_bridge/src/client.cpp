/*
 * @Brief: 
 * @Author: Baozhe ZHANG 
 * @Date: 2023-07-19 15:17:56 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-07-19 19:49:38
 */

#include <iostream>
#include <thread>
#include <chrono>

#include <steam/steamnetworkingsockets.h>
#include <steam/isteamnetworkingsockets.h>
#include <steam/isteamnetworkingutils.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/serialized_message.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

DEFINE_string(ip, "127.0.0.1", "ip");
DEFINE_uint32(port, 8888, "port");

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

class MsgClient
{
 public: 
  MsgClient() : spinner_(1)
  {
    spinner_.start();
  }
  void run(const SteamNetworkingIPAddr &server_addr)
  {
    interface_ptr_ = SteamNetworkingSockets();
    std::cout << "Hi" << std::endl;

    // Start connecting
		char szAddr[ SteamNetworkingIPAddr::k_cchMaxString ];
    std::cout << "Hi" << std::endl;
		server_addr.ToString( szAddr, sizeof(szAddr), true );
    std::cout << "Hi" << std::endl;
		// printf( "Connecting to server at %s", szAddr );
    std::cout << "Connecting to " << szAddr << std::endl;
		SteamNetworkingConfigValue_t opt;
		opt.SetPtr( k_ESteamNetworkingConfig_Callback_ConnectionStatusChanged, (void*)SteamNetConnectionStatusChangedCallback );
		h_connection_ = interface_ptr_->ConnectByIPAddress( server_addr, 1, &opt );
		if ( h_connection_ == k_HSteamNetConnection_Invalid )
			std::cerr <<  "Failed to create connection" << std::endl;

		while (true)
		{
			pollConnectionStateChanges();
			// std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
		}
  }
 private: 
  HSteamNetConnection h_connection_; // only one server
  ISteamNetworkingSockets *interface_ptr_;

  ros::NodeHandle nh_ = ros::NodeHandle();
  ros::Subscriber str_sub1_ = 
      nh_.subscribe<sensor_msgs::Imu>("/1/string", 1, 
      &MsgClient::cb, this, ros::TransportHints().tcpNoDelay());
  ros::AsyncSpinner spinner_;
  
  void cb(const sensor_msgs::Imu::ConstPtr &msg_ptr) {
    std::cout << "received" << std::endl;
    uint32_t msg_size = ros::serialization::serializationLength(*msg_ptr);
    std::vector<uint8_t> data(msg_size);
    ros::serialization::OStream stream(data.data(), msg_size);
    ros::serialization::serialize(stream, *msg_ptr); 
    interface_ptr_->SendMessageToConnection(h_connection_, 
        (void *)data.data(), msg_size, 
        k_nSteamNetworkingSend_Reliable, 
        nullptr);
    std::cout << "sent" << std::endl;
  }

  void onSteamNetConnectionStatusChanged( SteamNetConnectionStatusChangedCallback_t *pInfo )
	{
		assert( pInfo->m_hConn == h_connection_ || h_connection_ == k_HSteamNetConnection_Invalid );

		// What's the state of the connection?
		switch ( pInfo->m_info.m_eState )
		{
			case k_ESteamNetworkingConnectionState_None:
				// NOTE: We will get callbacks here when we destroy connections.  You can ignore these.
				break;

			case k_ESteamNetworkingConnectionState_ClosedByPeer:
			case k_ESteamNetworkingConnectionState_ProblemDetectedLocally:
			{

				// Print an appropriate message
				if ( pInfo->m_eOldState == k_ESteamNetworkingConnectionState_Connecting )
				{
					// Note: we could distinguish between a timeout, a rejected connection,
					// or some other transport problem.
					printf( "We sought the remote host, yet our efforts were met with defeat.  (%s)", pInfo->m_info.m_szEndDebug );
				}
				else if ( pInfo->m_info.m_eState == k_ESteamNetworkingConnectionState_ProblemDetectedLocally )
				{
					printf( "Alas, troubles beset us; we have lost contact with the host.  (%s)", pInfo->m_info.m_szEndDebug );
				}
				else
				{
					// NOTE: We could check the reason code for a normal disconnection
					printf( "The host hath bidden us farewell.  (%s)", pInfo->m_info.m_szEndDebug );
				}

				// Clean up the connection.  This is important!
				// The connection is "closed" in the network sense, but
				// it has not been destroyed.  We must close it on our end, too
				// to finish up.  The reason information do not matter in this case,
				// and we cannot linger because it's already closed on the other end,
				// so we just pass 0's.
				interface_ptr_->CloseConnection( pInfo->m_hConn, 0, nullptr, false );
				h_connection_ = k_HSteamNetConnection_Invalid;
				break;
			}

			case k_ESteamNetworkingConnectionState_Connecting:
				// We will get this callback when we start connecting.
				// We can ignore this.
				break;

			case k_ESteamNetworkingConnectionState_Connected:
				printf( "Connected to server OK" );
				break;

			default:
				// Silences -Wswitch
				break;
		}
	}

  static MsgClient *callback_instance_ptr_; 
  static void SteamNetConnectionStatusChangedCallback( SteamNetConnectionStatusChangedCallback_t *pInfo )
	{
		callback_instance_ptr_->onSteamNetConnectionStatusChanged( pInfo );
	}

  void pollConnectionStateChanges()
  {
    callback_instance_ptr_ = this;
    interface_ptr_->RunCallbacks();
  }
}; 

MsgClient *MsgClient::callback_instance_ptr_ = nullptr;

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "client");

  SteamNetworkingIPAddr addrServer; addrServer.Clear();
  InitSteamDatagramConnectionSockets();

  if (!addrServer.ParseString(FLAGS_ip.c_str())) {
    std::cerr << "Wrong ip format" << std::endl;
    exit(1);
  }
  addrServer.m_port = FLAGS_port;
  std::cout << "hi" << std::endl;

  MsgClient client;
  client.run(addrServer);
}