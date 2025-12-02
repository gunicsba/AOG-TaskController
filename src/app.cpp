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
#include "isobus/isobus/isobus_diagnostic_protocol.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
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
	ourNAME.set_arbitrary_address_capable(true); // Allow arbitrary address claim
	ourNAME.set_industry_group(2);
	ourNAME.set_device_class(0);
	ourNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
	ourNAME.set_identity_number(20);
	ourNAME.set_ecu_instance(0);
	ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
	ourNAME.set_device_class_instance(0);
	ourNAME.set_manufacturer_code(1407);

	// Create separate NAME objects for Task Controller, Tractor ECU, and VT Client
	isobus::NAME tcNAME = ourNAME; // Copy the base configuration
	tcNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
	
	isobus::NAME tecuNAME = ourNAME; // Copy the base configuration
	tecuNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TractorECU));
	tecuNAME.set_ecu_instance(1);
	
	// Create a separate control function for VT client with a different function code
	// Use RateControl (or any non-TC function) to avoid confusing the VT server
	isobus::NAME vtClientNAME = ourNAME;
	vtClientNAME.set_function_code(0x17);//static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));  TODO: need a good function code.
	vtClientNAME.set_ecu_instance(2);

	// Create separate control functions for Task Controller, Tractor ECU, and VT Client
	// Don't request specific address for TC - let it claim any available address to avoid conflicts
	// Tractor ECU will use its preferred address
	// VT Client gets its own control function to avoid confusing the VT server
	// Create VT client CF first with preferred address 127 (to avoid conflicts with other ECUs)
	auto vtClientCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(vtClientNAME, 0, 127);
	std::cout << "[Init] VT Client control function created (prefer address 127), waiting 2 seconds..." << std::endl;
	
	// Update the network manager to process VT client CF claiming
	for (int i = 0; i < 20; i++) {
		isobus::CANNetworkManager::CANNetwork.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	// Create TC control function with arbitrary address claiming (no preferred address to avoid conflicts)
	std::shared_ptr<isobus::InternalControlFunction> tcCF = nullptr;
	if (true) { // Enable TC creation
		std::cout << "[Init] Creating Task Controller control function..." << std::endl;
		tcCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tcNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer);
		std::cout << "[Init] Task Controller control function created, waiting 2 seconds..." << std::endl;
		
		// Update the network manager to process TC CF claiming
		for (int i = 0; i < 20; i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	}

	// Create TECU control function 2 seconds after TC
	std::shared_ptr<isobus::InternalControlFunction> tecuCF = nullptr;
	if (tcCF) { // Only create TECU if TC was created
		std::cout << "[Init] Creating Tractor ECU control function..." << std::endl;
		tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU);
		std::cout << "[Init] Tractor ECU control function created, waiting 1.5 seconds..." << std::endl;
		
		// Update the network manager to process TECU CF claiming
		for (int i = 0; i < 15; i++) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	}

	diagnosticProtocol = std::make_unique<isobus::DiagnosticProtocol>(vtClientCF);
	diagnosticProtocol->initialize();

	diagnosticProtocol->set_product_identification_code("1234567890ABC");
	diagnosticProtocol->set_product_identification_brand("AgIsoStack++");
	diagnosticProtocol->set_product_identification_model("AOG-Task Controller");
	diagnosticProtocol->set_software_id_field(0, "Example 1.0.0");
	diagnosticProtocol->set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::HardwareID, "1234");
	diagnosticProtocol->set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::Location, "N/A");
	diagnosticProtocol->set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::ManufacturerName, "Open-Agriculture");
	diagnosticProtocol->set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::PartNumber, "1234");
	diagnosticProtocol->set_ecu_id_field(isobus::DiagnosticProtocol::ECUIdentificationFields::SerialNumber, "2");
	diagnosticProtocol->ControlFunctionFunctionalitiesMessageInterface.set_functionality_is_supported(isobus::ControlFunctionFunctionalities::Functionalities::MinimumControlFunction, 1, true);
	diagnosticProtocol->ControlFunctionFunctionalitiesMessageInterface.set_functionality_is_supported(isobus::ControlFunctionFunctionalities::Functionalities::UniversalTerminalWorkingSet, 1, true);



	// Wait for all address claiming processes to complete
	auto tcAddressClaimedFuture = std::async(std::launch::async, [&tcCF]() {
		// Wait up to 10 seconds for address validation
		auto startTime = std::chrono::steady_clock::now();
		while (tcCF && !tcCF->get_address_valid() && 
		       (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(10)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	});
	
	auto tecuAddressClaimedFuture = std::async(std::launch::async, [&tecuCF]() {
		// Wait up to 10 seconds for address validation
		auto startTime = std::chrono::steady_clock::now();
		while (tecuCF && !tecuCF->get_address_valid() && 
		       (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(10)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	});

	// Wait for address claiming with timeout
	if (tcAddressClaimedFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
		std::cout << "Warning: Task Controller address claiming timed out. The control function may not have claimed the desired address." << std::endl;
	}
	
	if (tecuAddressClaimedFuture.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
		std::cout << "Warning: Tractor ECU address claiming timed out. The control function may not have claimed the desired address." << std::endl;
	}
	
	// Check what addresses were actually claimed for Task Controller
	if (tcCF && tcCF->get_address_valid()) {
		std::cout << "Task Controller control function successfully claimed address: " << static_cast<int>(tcCF->get_address()) << " (0x" 
		          << std::hex << static_cast<int>(tcCF->get_address()) << std::dec << ")" << std::endl;
	} else {
		std::cout << "Warning: Task Controller control function address is not valid. This may cause communication issues." << std::endl;
	}
	
	// Check what addresses were actually claimed for Tractor ECU
	if (tecuCF && tecuCF->get_address_valid()) {
		std::cout << "Tractor ECU control function successfully claimed address: " << static_cast<int>(tecuCF->get_address()) << " (0x" 
		          << std::hex << static_cast<int>(tecuCF->get_address()) << std::dec << ")" << std::endl;
	} else {
		std::cout << "Warning: Tractor ECU control function address is not valid. This may cause communication issues." << std::endl;
	}
	
	// If any critical function fails, probably the update thread is not started
	if ((tcCF && !tcCF->get_address_valid()) || (tecuCF && !tecuCF->get_address_valid()))
	{
		std::cout << "Failed to claim address for one or more critical control functions. The control function(s) might be invalid." << std::endl;
		return false;
	}

	// Store the control functions in member variables
	tcControlFunction = tcCF;
	tecuControlFunction = tecuCF;

	// Initialize Virtual Terminal Client
	try {
		// Create a partnered control function to represent the VT server we want to connect to
		const isobus::NAMEFilter filterVirtualTerminal(
			isobus::NAME::NAMEParameters::FunctionCode, 
			static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
		const std::vector<isobus::NAMEFilter> vtNameFilters = { filterVirtualTerminal };
		auto partnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);



		// Create VT client with its own dedicated control function (not the TC CF)
		vtClient = std::make_shared<isobus::VirtualTerminalClient>(partnerVT, vtClientCF);
		
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
				if (vtClient) vtClient->set_object_pool(0, objectPool.data(), objectPool.size(), objectPoolHash);
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
		if (vtClient) vtClient->get_vt_soft_key_event_dispatcher().add_listener(
			[this](const isobus::VirtualTerminalClient::VTKeyEvent &event) {
				// Handle your soft key presses
				std::cout << "VT Soft Key Event: Key Number " << static_cast<int>(event.keyNumber) 
				          << ", Key Event " << static_cast<int>(event.keyEvent) 
				          << ", Object ID " << event.objectID << std::endl;
				handle_vt_key_events(event);
			});

		// Handle button events
		if (vtClient) vtClient->get_vt_button_event_dispatcher().add_listener(
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

		// DON'T initialize yet - wait for VT partner to be discovered
		// vtClient->initialize(true); will be called in update() when VT partner is detected
		vtClientStarted = false; // Will be set to true after initialization
		std::cout << "Virtual Terminal Client created, waiting for VT partner discovery before initializing." << std::endl;
		
		// Create VT update helper for automatic change tracking
		vtUpdateHelper = std::make_unique<isobus::VirtualTerminalClientUpdateHelper>(vtClient);
		// Add tracked numeric values
		vtUpdateHelper->add_tracked_numeric_value(VTSpeedValue, 0); // speed value, default 0
		vtUpdateHelper->add_tracked_numeric_value(VTXteValue, 0); // XTE value, default 0
		vtUpdateHelper->initialize();
		std::cout << "VirtualTerminalClientUpdateHelper created and initialized." << std::endl;
	}
	catch (const std::exception& e) {
		std::cout << "Failed to initialize Virtual Terminal Client: " << e.what() << std::endl;
		vtClient.reset();
	}

	// Use the Task Controller control function for the server (as it needs to send TC messages)
	if (tcCF) {
		std::cout << "[Init] Creating Task Controller Server..." << std::endl;
		tcServer = std::make_shared<MyTCServer>(tcCF);
		tcServer->initialize();
		tcServerStarted = true;
		std::cout << "[Init] Task Controller Server created and initialized." << std::endl;
	} else {
		std::cout << "[Warning] TC Control Function not available, TC Server not created" << std::endl;
		tcServerStarted = false;
	}

	// Initialize Speed/NMEA interfaces on TECU control function
	if (tecuCF) {
		std::cout << "[Init] Creating Speed Messages Interface on TECU..." << std::endl;
		speedMessagesInterface = std::make_unique<isobus::SpeedMessagesInterface>(tecuCF, true, true, true, false);
		speedMessagesInterface->initialize();
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_implement_start_stop_operations_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::ImplementStartStopOperations::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_key_switch_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::KeySwitchState::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_operator_direction_reversed_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::OperatorDirectionReversed::NotAvailable);
		speedMessagesInterface->machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);

		std::cout << "[Init] Speed Messages Interface created and initialized." << std::endl;
		
		std::cout << "[Init] Creating NMEA2000 Message Interface on TECU..." << std::endl;
		nmea2000MessageInterface = std::make_unique<isobus::NMEA2000MessageInterface>(tecuCF, false, false, false, false, false, false, false);
		nmea2000MessageInterface->initialize();
		nmea2000MessageInterface->set_enable_sending_cog_sog_cyclically(true);
		std::cout << "[Init] NMEA2000 Message Interface created and initialized." << std::endl;
	} else {
		std::cout << "[Warning] TECU Control Function not available, Speed/NMEA interfaces not created" << std::endl;
	}

	std::cout << "Task controller server started." << std::endl;

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
				if(vtClient) {
					if(ss != lastSectionStates){
//						vtClient->send_change_string_value(VTSectionsFromAOGS, 16, ss.c_str());
						lastSectionStates = ss;
					}
				}
			
			if (tcServer)
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
					if (tcServer) {
						tcServer->set_left_tramline_state(leftTram);
						tcServer->set_right_tramline_state(rightTram);
						tcServer->update_tramline_states(leftTram, rightTram);
					}
				}
			}
		}
		else if (src == 0x7F && pgn == 0xF1) // 241 - Section Control
		{
			std::uint8_t sectionControlState = data[0];
			std::cout << "Received request from AOG to change section control state to " << (sectionControlState == 1 ? "enabled" : "disabled") << std::endl;
			if (tcServer)
				tcServer->update_section_control_enabled(sectionControlState == 1);
		}
		else if (src == 0x7F && pgn == 0xF2) // Process Data
		{
			auto identifier = static_cast<isobus::DataDescriptionIndex>(data[0] | (data[1] << 8));

			std::int32_t value = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
			if (identifier == isobus::DataDescriptionIndex::ActualSpeed)
			{
				std::uint16_t speed = std::abs(value);
				if (speedMessagesInterface) {
					auto direction = value < 0 ? isobus::SpeedMessagesInterface::MachineDirection::Reverse : isobus::SpeedMessagesInterface::MachineDirection::Forward;
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_direction_of_travel(direction);
					
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_speed(speed);

					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
				}
				if (nmea2000MessageInterface) {
					auto &cog_sog_message = nmea2000MessageInterface->get_cog_sog_transmit_message();
					cog_sog_message.set_sequence_id(nmea2000SequenceIdentifier++);
					cog_sog_message.set_speed_over_ground(speed/10);
					cog_sog_message.set_course_over_ground(0); // TODO: Implement course
					cog_sog_message.set_course_over_ground_reference(isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate::CourseOverGroundReference::NotApplicableOrNull);
				}
				if (vtUpdateHelper) {
				    vtUpdateHelper->set_numeric_value(VTSpeedValue, speed);
				}
			}
			else if (identifier == isobus::DataDescriptionIndex::GuidanceLineDeviation)
			{
				if (vtUpdateHelper) {
				     vtUpdateHelper->set_numeric_value(VTXteValue, value);
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
				
				if (tecuControlFunction && isobus::SystemTiming::time_expired_ms(lastXteTransmit, 1000))
				{
					if (isobus::CANNetworkManager::CANNetwork.send_can_message(0x1F903, xteData.data(), xteData.size(), tecuControlFunction))
					{
						lastXteTransmit = isobus::SystemTiming::get_timestamp_ms();
					}
				}
			}
			else if (static_cast<std::uint16_t>(identifier) == 597 /*isobus::DataDescriptionIndex::TotalDistance*/)
			{
				auto distance = static_cast<std::uint32_t>(value);
				if (speedMessagesInterface) {
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(distance);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(distance);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(distance);
				}
			}
		}
	};
	udpConnections->set_packet_handler(packetHandler);
	udpConnections->set_connection_status_handler([this](bool isConnected, const std::string& localAddress) {
		this->handle_connection_status_change(isConnected, localAddress);
	});
	udpConnections->open();

	std::cout << "UDP connections opened." << std::endl;

	return true;
}

bool Application::update()
{
	static std::uint32_t lastHeartbeatTransmit = 0;

	udpConnections->handle_address_detection();
	udpConnections->handle_incoming_packets();

	if (tcServer) {
		tcServer->request_measurement_commands();
		tcServer->update();
	}
	if (speedMessagesInterface) speedMessagesInterface->update();
	if (nmea2000MessageInterface) nmea2000MessageInterface->update();
	
	if (vtClient) {
		vtClient->update();
		// Update CAN network first - VT client needs this when running without separate thread
		isobus::CANNetworkManager::CANNetwork.update();
		
		// Initialize VT client when VT partner is discovered
		if (!vtClientStarted) {
			auto vtPartner = vtClient->get_partner_control_function();
			if (vtPartner && vtPartner->get_address_valid()) {
				std::cout << "[VT] Partner discovered at address " << static_cast<int>(vtPartner->get_address()) << ", waiting 5 seconds before initializing..." << std::endl;
				std::cout << "[VT] Initializing VT client..." << std::endl;
				vtClient->initialize(false); // false = NO separate thread, we'll call update() ourselves
				vtClientStarted = true;
				std::cout << "[VT] Virtual Terminal Client initialized (manual update mode)." << std::endl;
			}
		}
		
		// Mark VT client as ready after a short delay to ensure it's fully initialized
		static auto vtInitTime = std::chrono::steady_clock::now();
		if (vtClientStarted && !vtClientReady && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - vtInitTime).count() > 3) {
			vtClientReady = true;
			std::cout << "VT client is ready for updates" << std::endl;
		}
		
		// Log VT connection status periodically
		static std::uint32_t lastVTStatusLog = 0;
		if (isobus::SystemTiming::time_expired_ms(lastVTStatusLog, 5000)) {
			auto vtPartner = vtClient->get_partner_control_function();
			if (!vtClient->get_is_connected()) 
			{
				std::cout << "[VT] Waiting for connection to VT Server...";
				if (vtPartner->get_address_valid()) {
					std::cout << " (VT partner detected at address " << static_cast<int>(vtPartner->get_address()) << ")";
				} else {
					std::cout << " (VT partner not yet discovered)";
				}
				std::cout << std::endl;
			}
			lastVTStatusLog = isobus::SystemTiming::get_timestamp_ms();
		}
		
		// Send Address Claimed request to force VT to re-announce
		static std::uint32_t lastDiscoveryRequest = 0;
		static bool vtPartnerEverSeen = false;
		auto vtPartner = vtClient->get_partner_control_function();
		
		// Track if we've ever seen the VT partner
		if (vtPartner->get_address_valid()) {
			vtPartnerEverSeen = true;
		}
		
		if (is_vt_ready()) {
			if (lastConnectionState != lastVTConnectionState) {
				vtClient->send_change_background_colour(VTAogIPStr, lastConnectionState ? 2 : 12); // 2 = GREEN 12 = RED
				lastVTConnectionState = lastConnectionState;
			}
			if(!lastLocalAddress.empty())
				vtClient->send_change_string_value(VTAogIPStr, lastLocalAddress.length(), lastLocalAddress.c_str());
		}
	}

	if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
	{
	  if(tcServer) 
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
	  }
		lastHeartbeatTransmit = isobus::SystemTiming::get_timestamp_ms();
	}

	return true;
}

void Application::stop()
{
	// Clean up VT client if it exists
	if (vtClient)
	{
		vtClient->terminate();
		vtClient.reset();
	}
	if (nullptr != diagnosticProtocol)
	{
		diagnosticProtocol->terminate();
	}
	
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}


void Application::handle_connection_status_change(bool isConnected, const std::string& localAddress)
{
	// Store the connection state for periodic updates
	lastConnectionState = isConnected;
	lastLocalAddress = localAddress;
	
	if (isConnected) {
		std::cout << "AgOpenGPS connection restored! " << localAddress << std::endl;
	} else {
		std::cout << "AgOpenGPS connection lost!" << std::endl;
	}
	
	// Defer VT UI updates to update() thread to avoid cross-thread VT calls
}

void Application::perform_bus_scan_to_vt()
{
	if (!is_vt_ready()) {
		std::cout << "[Bus Scan] VT not ready, cannot display results" << std::endl;
		return;
	}
	
	std::cout << "[Bus Scan] Scanning CAN bus for active control functions..." << std::endl;
	
	// Build a formatted string with all active control functions
	// Limit to 255 characters for VT display
	std::string result;
	auto controlFunctions = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);
	
	if (controlFunctions.empty()) {
		result = "No CFs found";
	} else {
		// Pretty format: "Addr:Func:NAME\nAddr:Func:NAME\n..." with newlines
		for (const auto& cf : controlFunctions) {
			if (cf && cf->get_address_valid()) {
				auto name = cf->get_NAME();
				std::uint8_t addr = cf->get_address();
				std::uint8_t func = static_cast<std::uint8_t>(name.get_function_code());
				std::uint64_t fullName = name.get_full_name();
				
				// Format as "Addr:Func:NAME\n" (with newline for better VT display)
				char entry[32];
				snprintf(entry, sizeof(entry), "%u:%u:%llx\n", addr, func, fullName);
				
				// Check if adding this entry would exceed 255 chars
				if (result.length() + strlen(entry) > 255) {
					result += "...";
					break;
				}
				
				result += entry;
			}
		}
		
		// Remove trailing newline if any
		if (!result.empty() && result.back() == '\n') {
			result.pop_back();
		}
	}
	
	// Also print detailed info to console
	std::cout << "[Bus Scan] === Control Functions on CAN Bus ===" << std::endl;
	for (const auto& cf : controlFunctions) {
		if (cf && cf->get_address_valid()) {
			auto name = cf->get_NAME();
			std::cout << "  - Address: " << static_cast<int>(cf->get_address()) 
			          << ", Function: " << static_cast<int>(name.get_function_code())
			          << " (" << (name.get_function_code() == static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal) ? "VT" : "Other") << ")"
			          << ", NAME: " << std::hex << name.get_full_name() << std::dec << std::endl;
		}
	}
	std::cout << "[Bus Scan] ================================" << std::endl;
	
	std::cout << "[Bus Scan] Found " << controlFunctions.size() << " control functions" << std::endl;
	std::cout << "[Bus Scan] Result (" << result.length() << " chars): " << result << std::endl;
	
	// Send to VT
	vtClient->send_change_string_value(VTControlFunctionsStr, result.length(), result.c_str());
}

void Application::perform_implement_size_display()
{
	if (!is_vt_ready()) {
		std::cout << "[Implement Size] VT not ready, cannot display results" << std::endl;
		return;
	}
	
	if (!tcServer) {
		std::cout << "[Implement Size] TC Server not available" << std::endl;
		vtClient->send_change_string_value(VTControlFunctionsStr, 19, "No TC Server found");
		return;
	}
	
	std::cout << "[Implement Size] Displaying implement geometry..." << std::endl;
	
	// Get all TC clients
	auto clients = tcServer->get_clients();
	if (clients.empty()) {
		std::cout << "[Implement Size] No TC clients connected" << std::endl;
		vtClient->send_change_string_value(VTControlFunctionsStr, 22, "No TC clients connected");
		implementSizePage = 0;
		return;
	}
	
	// Use the first client (assuming single implement)
	auto& clientState = clients.begin()->second;
	auto& pool = clientState.get_pool();
	auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool);
	
	// Build pages
	std::vector<std::string> pages;
	
	// Page 0: Summary
	std::string summary;
	std::uint16_t totalSections = 0;
	std::uint16_t totalBooms = implement.booms.size();
	std::uint16_t totalSubBooms = 0;
	std::vector<std::uint16_t> sectionWidths;
	
	for (const auto &boom : implement.booms) {
		for (const auto &subBoom : boom.subBooms) {
			totalSubBooms++;
			for (const auto &section : subBoom.sections) {
				totalSections++;
				sectionWidths.push_back(static_cast<std::uint16_t>(section.width_mm.get() / 10)); // Convert mm to cm
			}
		}
		for (const auto &section : boom.sections) {
			totalSections++;
			sectionWidths.push_back(static_cast<std::uint16_t>(section.width_mm.get() / 10)); // Convert mm to cm
		}
	}
	
	// Build summary page
	char summaryBuf[256];
	snprintf(summaryBuf, sizeof(summaryBuf), "Booms:%u SubBooms:%u Sections:%u\nWidths(cm):",
	         totalBooms, totalSubBooms, totalSections);
	summary = summaryBuf;
	
	// Add section widths
	for (size_t i = 0; i < sectionWidths.size(); i++) {
		char widthStr[16];
		snprintf(widthStr, sizeof(widthStr), "%s%u", (i > 0 ? "," : ""), sectionWidths[i]);
		if (summary.length() + strlen(widthStr) > 200) {
			summary += "...";
			break;
		}
		summary += widthStr;
	}
	pages.push_back(summary);
	
	// Detailed pages: Section by section
	std::uint16_t sectionIndex = 0;
	for (const auto &boom : implement.booms) {
		for (const auto &subBoom : boom.subBooms) {
			for (const auto &section : subBoom.sections) {
				char sectionInfo[256];
				snprintf(sectionInfo, sizeof(sectionInfo),
				         "Sec %u: ID=%u\nX=%d Y=%d Z=%d\nW=%d mm (%u cm)",
				         sectionIndex + 1,
				         static_cast<unsigned>(section.elementNumber),
				         section.xOffset_mm.get(),
				         section.yOffset_mm.get(),
				         section.zOffset_mm.get(),
				         section.width_mm.get(),
				         section.width_mm.get() / 10);
				pages.push_back(std::string(sectionInfo));
				sectionIndex++;
			}
		}
		// Process boom sections (outside subBoom loop, still inside boom loop)
		for (size_t i = 0; i < boom.sections.size(); i++) {
			const auto &section = boom.sections[i];
			char sectionInfo[256];
			snprintf(sectionInfo, sizeof(sectionInfo),
			         "Sec %u: ID=%u\nX=%d Y=%d Z=%d\nW=%d mm (%u cm)",
			         sectionIndex + 1,
			         static_cast<unsigned>(section.elementNumber),
			         section.xOffset_mm.get(),
			         section.yOffset_mm.get(),
			         section.zOffset_mm.get(),
			         section.width_mm.get(),
			         section.width_mm.get() / 10);
			pages.push_back(std::string(sectionInfo));
			sectionIndex++;
		}
	}
	
	// Cycle through pages
	if (pages.empty()) {
		vtClient->send_change_string_value(VTControlFunctionsStr, 21, "No implement geometry");
		implementSizePage = 0;
		return;
	}
	
	implementSizePage = implementSizePage % pages.size();
	std::string& currentPage = pages[implementSizePage];
	
	// Add page indicator if multiple pages
	if (pages.size() > 1) {
		char pageIndicator[32];
		snprintf(pageIndicator, sizeof(pageIndicator), "\n[%u/%zu]", implementSizePage + 1, pages.size());
		if (currentPage.length() + strlen(pageIndicator) <= 255) {
			currentPage += pageIndicator;
		}
	}
	
	std::cout << "[Implement Size] Page " << (implementSizePage + 1) << "/" << pages.size() << ": " << currentPage << std::endl;
	
	// Send to VT
	vtClient->send_change_string_value(VTControlFunctionsStr, currentPage.length(), currentPage.c_str());
	
	// Advance to next page for next click
	implementSizePage++;
}

void Application::set_output_number_value(std::uint16_t objectID, std::uint32_t value)
{
	// Change a numeric output object in the Virtual Terminal
	if (is_vt_ready()) {
		vtClient->send_change_numeric_value(objectID, value);
		std::cout << "Set OutputNumber object " << objectID << " to value " << value << std::endl;
	}
}

void Application::handle_vt_key_events(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	// Handle VT key events
	// This is where you would implement your UI logic
	std::cout << "Handling VT key event: Key Number " << static_cast<int>(event.keyNumber) 
	          << ", Key Event " << static_cast<int>(event.keyEvent) 
	          << ", Object ID " << event.objectID << std::endl;
	
	// Check if this is the ListActiveControlFunctions soft key
	if (event.objectID == ListActiveControlFunctions && 
	    event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
	{
		perform_bus_scan_to_vt();
		return;
	}
	
	// Check if this is the ListImplementSize soft key
	if (event.objectID == ListImplementSize && 
	    event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
	{
		perform_implement_size_display();
		return;
	}
	
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