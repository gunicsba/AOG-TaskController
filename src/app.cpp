/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "app.hpp"

#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include "task_controller.hpp"
#include "AOG_TC.iop.h"

#include <iostream>
#include <memory>
#include <thread>
#include <future>

using boost::asio::ip::udp;

Application::Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver) :
  canDriver(canDriver)
{
}

bool Application::initialize()
{
	settings->load();
	if (nullptr == canDriver)
	{
		std::cout << "Unable to find a CAN driver. Please make sure the selected driver is installed." << std::endl;
		return false;
	}
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		std::cout << "Failed to start CAN hardware interface." << std::endl;
		return false;
	}

	isobus::NAME ourNAME(0);

	//! Make sure you change these for your device!!!!
	ourNAME.set_arbitrary_address_capable(true);
	ourNAME.set_industry_group(2);
	ourNAME.set_device_class(0);
	ourNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
	ourNAME.set_identity_number(20);
	ourNAME.set_ecu_instance(0);
	ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
	ourNAME.set_device_class_instance(0);
	ourNAME.set_manufacturer_code(1407);

	// Create separate NAME objects for Task Controller, Tractor ECU, and Virtual Terminal
	isobus::NAME tcNAME = ourNAME; // Copy the base configuration
	tcNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
	
	isobus::NAME tecuNAME = ourNAME; // Copy the base configuration
	tecuNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TractorECU));
	
	isobus::NAME vtNAME = ourNAME; // Copy the base configuration
	vtNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));

	// Create separate control functions for Task Controller, Tractor ECU, and Virtual Terminal
	// Task Controller will use address 0x26 (38) as required
	// Tractor ECU will use a different address to avoid conflicts
	// Virtual Terminal will use its preferred address
	auto tcCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tcNAME, 0, 0x26); // Task Controller address
	auto tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU); // TECU preferred address
	auto vtCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(vtNAME, 0); // VT with no specific address
	
	// Wait for all address claiming processes to complete
	auto tcAddressClaimedFuture = std::async(std::launch::async, [&tcCF]() {
		// Wait up to 10 seconds for address validation
		auto startTime = std::chrono::steady_clock::now();
		while (!tcCF->get_address_valid() && 
		       (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(10)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	});
	
	auto tecuAddressClaimedFuture = std::async(std::launch::async, [&tecuCF]() {
		// Wait up to 10 seconds for address validation
		auto startTime = std::chrono::steady_clock::now();
		while (!tecuCF->get_address_valid() && 
		       (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(10)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	});
	
	auto vtAddressClaimedFuture = std::async(std::launch::async, [&vtCF]() {
		// Wait up to 10 seconds for address validation
		auto startTime = std::chrono::steady_clock::now();
		while (!vtCF->get_address_valid() && 
		       (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(10)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	});

	// Wait for address claiming with timeout
	if (tcAddressClaimedFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
		std::cout << "Warning: Task Controller address claiming timed out. The control function may not have claimed the desired address." << std::endl;
	}
	
	if (tecuAddressClaimedFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
		std::cout << "Warning: Tractor ECU address claiming timed out. The control function may not have claimed the desired address." << std::endl;
	}
	
	if (vtAddressClaimedFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
		std::cout << "Warning: Virtual Terminal address claiming timed out. The control function may not have claimed the desired address." << std::endl;
	}
	
	// Check what addresses were actually claimed for Task Controller
	if (tcCF->get_address_valid()) {
		std::cout << "Task Controller control function successfully claimed address: " << static_cast<int>(tcCF->get_address()) << " (0x" 
		          << std::hex << static_cast<int>(tcCF->get_address()) << std::dec << ")" << std::endl;
		
		// If the address is not what we wanted, log a warning
		if (tcCF->get_address() != 0x26) {
			std::cout << "Warning: Task Controller control function claimed address " << static_cast<int>(tcCF->get_address()) 
			          << " instead of the requested address 38 (0x26)" << std::endl;
			std::cout << "This may cause communication issues with implements expecting messages from address 38." << std::endl;
		}
	} else {
		std::cout << "Warning: Task Controller control function address is not valid. This may cause communication issues." << std::endl;
	}
	
	// Check what addresses were actually claimed for Tractor ECU
	if (tecuCF->get_address_valid()) {
		std::cout << "Tractor ECU control function successfully claimed address: " << static_cast<int>(tecuCF->get_address()) << " (0x" 
		          << std::hex << static_cast<int>(tecuCF->get_address()) << std::dec << ")" << std::endl;
	} else {
		std::cout << "Warning: Tractor ECU control function address is not valid. This may cause communication issues." << std::endl;
	}
	
	// Check what addresses were actually claimed for Virtual Terminal
	if (vtCF->get_address_valid()) {
		std::cout << "Virtual Terminal control function successfully claimed address: " << static_cast<int>(vtCF->get_address()) << " (0x" 
		          << std::hex << static_cast<int>(vtCF->get_address()) << std::dec << ")" << std::endl;
	} else {
		std::cout << "Warning: Virtual Terminal control function address is not valid. This may cause communication issues." << std::endl;
	}

	// If any critical function fails, probably the update thread is not started
	if (!tcCF->get_address_valid() || !tecuCF->get_address_valid())
	{
		std::cout << "Failed to claim address for one or more critical control functions. The control function(s) might be invalid." << std::endl;
		return false;
	}

	// Store the control functions in member variables
	tcControlFunction = tcCF;
	tecuControlFunction = tecuCF;
	vtControlFunction = vtCF;

	// Initialize Virtual Terminal Client if the header exists
	// Note: This is conditional because we're not sure if the VT client is available in this version of AgIsoStack++
	#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
	try {
		// Create a partnered control function to represent the VT server we want to connect to
		const isobus::NAMEFilter filterVirtualTerminal(
			isobus::NAME::NAMEParameters::FunctionCode, 
			static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
		const std::vector<isobus::NAMEFilter> vtNameFilters = { filterVirtualTerminal };
		auto partnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

		// Create VT client
		vtClient = std::make_shared<isobus::VirtualTerminalClient>(partnerVT, vtCF);
		
		// Try to load object pool from different possible locations
		std::vector<std::string> possiblePaths = {
			"AOG_TC.iop",           // Current directory
			"src/AOG_TC.iop",       // src folder
			"../src/AOG_TC.iop",    // Relative path to src folder
			"./src/AOG_TC.iop"      // Alternative relative path
		};
		
		bool objectPoolLoaded = false;
		for (const auto& path : possiblePaths) {
			std::cout << "Trying to load object pool from: " << path << std::endl;
			objectPool = isobus::IOPFileInterface::read_iop_file(path);
			if (!objectPool.empty()) {
				std::string objectPoolHash = isobus::IOPFileInterface::hash_object_pool_to_version(objectPool);
				vtClient->set_object_pool(0, objectPool.data(), objectPool.size(), objectPoolHash);
				std::cout << "Successfully loaded object pool from " << path << " with " << objectPool.size() << " bytes" << std::endl;
				objectPoolLoaded = true;
				break;
			} else {
				std::cout << "Failed to load object pool from: " << path << std::endl;
			}
		}
		
		if (!objectPoolLoaded) {
			std::cout << "Warning: Failed to load object pool from any of the expected locations:" << std::endl;
			for (const auto& path : possiblePaths) {
				std::cout << "  - " << path << std::endl;
			}
			std::cout << "The VT client will initialize without an object pool." << std::endl;
		}
		
		// Handle soft key events
		vtClient->get_vt_soft_key_event_dispatcher().add_listener(
			[this](const isobus::VirtualTerminalClient::VTKeyEvent &event) {
				// Handle your soft key presses
				std::cout << "VT Soft Key Event: Key Number " << static_cast<int>(event.keyNumber) 
				          << ", Key Event " << static_cast<int>(event.keyEvent) 
				          << ", Object ID " << event.objectID << std::endl;
				handle_vt_key_events(event);
			});

		// Handle button events
		vtClient->get_vt_button_event_dispatcher().add_listener(
			[this](const isobus::VirtualTerminalClient::VTKeyEvent &event) {
				// Handle your button presses
				std::cout << "VT Button Event: Key Number " << static_cast<int>(event.keyNumber) 
				          << ", Key Event " << static_cast<int>(event.keyEvent) 
				          << ", Object ID " << event.objectID << std::endl;
				handle_vt_key_events(event);
			});

		// Handle numeric value events
		vtClient->get_vt_change_numeric_value_event_dispatcher().add_listener(
			[this](const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event) {
				std::cout << "VT Numeric Value Event: Object ID " << event.objectID
				          << ", Value " << event.value << std::endl;
				handle_numeric_value_events(event);
			});

		// Initialize and start
		vtClient->initialize(true); // true = spawns own thread
		std::cout << "Virtual Terminal Client initialized successfully." << std::endl;
	}
	catch (const std::exception& e) {
		std::cout << "Failed to initialize Virtual Terminal Client: " << e.what() << std::endl;
		vtClient.reset();
	}
	#endif // ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE

	// Use the Task Controller control function for the server (as it needs to send TC messages)
	tcServer = std::make_shared<MyTCServer>(tcCF);
	auto &languageInterface = tcServer->get_language_command_interface();
	languageInterface.set_language_code("en"); // This is the default, but you can change it if you want
	languageInterface.set_country_code("US"); // This is the default, but you can change it if you want
	tcServer->initialize();
	tcServer->set_task_totals_active(true); // TODO: make this dynamic based on status in AOG

	// Initialize speed and distance messages
	speedMessagesInterface = std::make_unique<isobus::SpeedMessagesInterface>(tecuCF, true, true, true, false); //TODO: make configurable whether to send these messages
	speedMessagesInterface->initialize();
	nmea2000MessageInterface = std::make_unique<isobus::NMEA2000MessageInterface>(tecuCF, false, false, false, false, false, false, false);
	nmea2000MessageInterface->initialize();
	nmea2000MessageInterface->set_enable_sending_cog_sog_cyclically(true); // TODO: make configurable whether to send these messages

	speedMessagesInterface->wheelBasedSpeedTransmitData.set_implement_start_stop_operations_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::ImplementStartStopOperations::NotAvailable);
	speedMessagesInterface->wheelBasedSpeedTransmitData.set_key_switch_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::KeySwitchState::NotAvailable);
	speedMessagesInterface->wheelBasedSpeedTransmitData.set_operator_direction_reversed_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::OperatorDirectionReversed::NotAvailable);
	speedMessagesInterface->machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);

	std::cout << "Task controller server started." << std::endl;

	static std::uint8_t xteSid = 0;
	static std::uint32_t lastXteTransmit = 0;

	auto packetHandler = [this, tcCF](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		if (src == 0x7F && pgn == 0xFE) // 254 - Steer Data
		{
			// TODO: hack to get desired section states. probably want to make a new pgn later when we need more than 16 sections
			std::vector<bool> sectionStates;
			std::string ss = "";
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[6] & (1 << i));
				ss += std::to_string((data[6] & (1 << i))!=0);
			}
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[7] & (1 << i));
				ss += std::to_string((data[7] & (1 << i))!=0);
			}
			if(vtClient)
				vtClient->send_change_string_value(VTSectionsFromAOGS, 16, ss.c_str());

			tcServer->update_section_states(sectionStates);
		}
		if (src == 0x7F && pgn == 0xEF) // 239 - Machine Data
		{
			// Tramline data is at byte 8 (data[3] in our span)
			// Bit 0 = left tramline, Bit 1 = right tramline
			if (data.size() > 3)
			{
				bool leftTram = (data[3] & 0x01) != 0;
				bool rightTram = (data[3] & 0x02) != 0;
				static bool prevLeftTram = false;
				static bool prevRightTram = false;
				if ((leftTram != prevLeftTram) || (rightTram != prevRightTram))
				{
					std::cout << "AOG tramline change detected: left=" << (leftTram ? "ON" : "OFF")
					          << ", right=" << (rightTram ? "ON" : "OFF") << std::endl;
					prevLeftTram = leftTram;
					prevRightTram = rightTram;
					tcServer->set_left_tramline_state(leftTram);
					tcServer->set_right_tramline_state(rightTram);
					tcServer->update_tramline_states(leftTram, rightTram);
				}
			}
		}
		else if (src == 0x7F && pgn == 0xF1) // 241 - Section Control
		{
			std::uint8_t sectionControlState = data[0];
			std::cout << "Received request from AOG to change section control state to " << (sectionControlState == 1 ? "enabled" : "disabled") << std::endl;
			tcServer->update_section_control_enabled(sectionControlState == 1);
		}
		else if (src == 0x7F && pgn == 0xF2) // Process Data
		{
			auto identifier = static_cast<isobus::DataDescriptionIndex>(data[0] | (data[1] << 8));

			std::int32_t value = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
			if (identifier == isobus::DataDescriptionIndex::ActualSpeed)
			{
				std::uint16_t speed = std::abs(value);
				auto direction = value < 0 ? isobus::SpeedMessagesInterface::MachineDirection::Reverse : isobus::SpeedMessagesInterface::MachineDirection::Forward;
				speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
				speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
				speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_direction_of_travel(direction);

				speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_speed(speed);
				speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_speed(speed);
				speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_speed(speed);

				speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
				speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
				speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
				if (vtClient) {
					vtClient->send_change_numeric_value(VTSpeedValue, speed);
					if(speed > 100) 
						vtClient->send_change_string_value(VTWorkingSetStatusStr, 2, "OK");
					else 
						vtClient->send_change_string_value(VTWorkingSetStatusStr, 3, "LOW");
				}
				auto &cog_sog_message = nmea2000MessageInterface->get_cog_sog_transmit_message();
				cog_sog_message.set_sequence_id(nmea2000SequenceIdentifier++);
				cog_sog_message.set_speed_over_ground(speed/10);
				cog_sog_message.set_course_over_ground(0); // TODO: Implement course
				cog_sog_message.set_course_over_ground_reference(isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate::CourseOverGroundReference::NotApplicableOrNull);
			}
			else if (identifier == isobus::DataDescriptionIndex::GuidanceLineDeviation)
			{
				if (vtClient) {
					vtClient->send_change_numeric_value(VTXteValue, value);
				}
				std::int32_t xte = value / 1000; // Convert from mm to m
				static const std::uint8_t xteMode = 0b00000001;
				xteSid = xteSid % 253 + 1;

				std::uint8_t status = 0; // TODO: navigation terminated status

				std::array<std::uint8_t, 8> xteData = {
					xteSid, // Sequence ID
					static_cast<std::uint8_t>(xteMode | 0b00110000 | (status == 1 ? 0b00000000 : 0b01000000)), // XTE mode (4 bits) + Reserved (2 bits set to 1) + Navigation Terminated (2 bits)
					static_cast<std::uint8_t>(xte & 0xFF), // XTE LSB
					static_cast<std::uint8_t>((xte >> 8) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 16) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 24) & 0xFF), // XTE MSB
					0xFF, // Reserved byte 1 (all bits set to 1)
					0xFF // Reserved byte 2 (all bits set to 1)
				};
				if (isobus::SystemTiming::time_expired_ms(lastXteTransmit, 1000)) // Transmit every second
				{
					if (isobus::CANNetworkManager::CANNetwork.send_can_message(0x1F903, xteData.data(), xteData.size(), tcCF))
					{
						lastXteTransmit = isobus::SystemTiming::get_timestamp_ms();
					}
				}
			}
			else if (static_cast<std::uint16_t>(identifier) == 597 /*isobus::DataDescriptionIndex::TotalDistance*/)
			{
				auto distance = static_cast<std::uint32_t>(value);
				speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(distance);
				speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(distance);
				speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(distance);
			}
		}
	};
	udpConnections->set_packet_handler(packetHandler);
	udpConnections->open();

	std::cout << "UDP connections opened." << std::endl;

	return true;
}

bool Application::update()
{
	static std::uint32_t lastHeartbeatTransmit = 0;

	udpConnections->handle_address_detection();
	udpConnections->handle_incoming_packets();

	tcServer->request_measurement_commands();
	tcServer->update();
	speedMessagesInterface->update();
	nmea2000MessageInterface->update();
	
	#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
	if (vtClient) {
		// Update the VT client
		// Note: The VT client runs in its own thread, so this is just for any additional logic
		// You might want to update numeric values, strings, etc. based on your application state
	}
	#endif // ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE

	if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
	{
		for (auto &client : tcServer->get_clients())
		{
			auto &state = client.second;
			std::vector<uint8_t> data = { state.is_section_control_enabled(), state.get_number_of_sections() };

			std::uint8_t sectionIndex = 0;
			while (sectionIndex < state.get_number_of_sections())
			{
				std::uint8_t byte = 0;
				for (std::uint8_t i = 0; i < 8; i++)
				{
					if (sectionIndex < state.get_number_of_sections())
					{
						byte |= (state.get_section_actual_state(sectionIndex) == SectionState::ON) << i;
						sectionIndex++;
					}
				}
				data.push_back(byte);
			}
			udpConnections->send(0x80, 0xF0, data);
		}
		lastHeartbeatTransmit = isobus::SystemTiming::get_timestamp_ms();
	}

	return true;
}

void Application::stop()
{
	// Clean up VT client if it exists
	#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
	if (vtClient) {
		vtClient->terminate();
		vtClient.reset();
	}
	#endif // ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
	
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}

#ifdef ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE
void Application::set_output_number_value(std::uint16_t objectID, std::uint32_t value)
{
	// Change a numeric output object in the Virtual Terminal
	if (vtClient) {
		vtClient->send_change_numeric_value(objectID, value);
		std::cout << "Set OutputNumber object " << objectID << " to value " << value << std::endl;
	} else {
		std::cout << "Virtual Terminal Client is not initialized" << std::endl;
	}
}

void Application::handle_vt_key_events(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	// Handle VT key events
	// This is where you would implement your UI logic
	std::cout << "Handling VT key event: Key Number " << static_cast<int>(event.keyNumber) 
	          << ", Key Event " << static_cast<int>(event.keyEvent) 
	          << ", Object ID " << event.objectID << std::endl;
	
	// Example implementation - you would replace this with your actual UI logic
	switch (event.keyNumber)
	{
		case 0: // Example: ACK key
			if (event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
			{
				std::cout << "ACK key pressed" << std::endl;
				// Handle ACK functionality
			}
			break;
			
		default:
			std::cout << "Unhandled key event for key number: " << static_cast<int>(event.keyNumber) << std::endl;
			break;
	}
}

void Application::handle_numeric_value_events(const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event)
{
	// Handle VT numeric value events
	// This is where you would implement your UI logic for numeric inputs
	std::cout << "Handling VT numeric value event: Object ID " << event.objectID 
	          << ", Value " << event.value << std::endl;
	
	// Example implementation - you would replace this with your actual UI logic
	switch (event.objectID)
	{
		case 1000: // Example object ID for a setting
			std::cout << "Setting value changed to: " << event.value << std::endl;
			// Handle the changed value
			break;
			
		default:
			std::cout << "Unhandled numeric value event for object ID: " << event.objectID << std::endl;
			break;
	}
}
#endif // ISOBUS_VIRTUAL_TERMINAL_CLIENT_AVAILABLE