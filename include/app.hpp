/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#pragma once

#include <memory>

#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

// Include necessary ISOBUS headers
#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

// Virtual Terminal Client
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_diagnostic_protocol.hpp"
#include "../src/AOG_TC.iop.h"  // Include the IOP header for object ID definitions

// Include Boost ASIO
#include <boost/asio.hpp>

#include <vector>
#include <cstdint>

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext;
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::InternalControlFunction> tcControlFunction;
	std::shared_ptr<isobus::InternalControlFunction> tecuControlFunction;
	
	std::shared_ptr<isobus::VirtualTerminalClient> vtClient;
	std::unique_ptr<isobus::DiagnosticProtocol> diagnosticProtocol;
	std::vector<std::uint8_t> objectPool;
	
	// VT Event handlers and helpers
	void set_output_number_value(std::uint16_t objectID, std::uint32_t value);
	void handle_vt_key_events(const isobus::VirtualTerminalClient::VTKeyEvent& event);
	void handle_numeric_value_events(const isobus::VirtualTerminalClient::VTChangeNumericValueEvent& event);
	void handle_connection_status_change(bool isConnected, const std::string& localAddress);
		
	// Check if VT client is ready to receive commands
	bool is_vt_ready() const { return vtClientReady && vtClient && vtClient->get_is_connected(); }
		
	// Store VT readiness state
	bool vtClientReady = false;
	// Track TC and VT startup state
	bool tcServerStarted = false;
	bool vtClientStarted = false;
		
	// Track last connection status for periodic updates
	bool lastConnectionState = false;
	std::string lastLocalAddress;
	bool lastVTConnectionState = false;  // Track last state we sent to VT
	std::uint32_t vtConnectedSinceMs = 0;
	std::uint32_t lastUdpReconnectMs = 0;
	
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::uint8_t nmea2000SequenceIdentifier = 0;
};