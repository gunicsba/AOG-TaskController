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

// Conditional include for Virtual Terminal Client
#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#endif

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
	std::shared_ptr<isobus::InternalControlFunction> vtControlFunction;
	
	#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
	std::shared_ptr<isobus::VirtualTerminalClient> vtClient;
	std::vector<std::uint8_t> objectPool;
	
	// VT Event handlers and helpers
	void set_output_number_value(std::uint16_t objectID, std::uint32_t value);
	void handle_vt_key_events(const isobus::VirtualTerminalClient::VTKeyEvent& event);
	void handle_numeric_value_events(const isobus::VirtualTerminalClient::VTChangeNumericValueEvent& event);
	#endif
	
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::uint8_t nmea2000SequenceIdentifier = 0;
};