/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "task_controller.hpp"
#include "settings.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <bitset>
#include <fstream>
#include <iostream>

// Define DDI 669 for Track Number Shift as it's not in the standard AgIsoStack++ library
constexpr std::uint16_t DDI_TRACK_NUMBER_SHIFT = 669;

void ClientState::set_number_of_sections(std::uint8_t number)
{
	numberOfSections = number;
	sectionSetpointStates.resize(number);
	sectionActualStates.resize(number);
}

void ClientState::set_section_setpoint_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionSetpointStates[section] = state;
	}
}

void ClientState::set_section_actual_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionActualStates[section] = state;
	}
}

std::uint8_t ClientState::get_number_of_sections() const
{
	return numberOfSections;
}

std::uint8_t ClientState::get_section_setpoint_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionSetpointStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

std::uint8_t ClientState::get_section_actual_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionActualStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

bool ClientState::is_any_section_setpoint_on() const
{
	for (std::uint8_t state : sectionSetpointStates)
	{
		if (state == SectionState::ON)
		{
			return true;
		}
	}
	return false;
}

bool ClientState::get_setpoint_work_state() const
{
	return setpointWorkState;
}

void ClientState::set_setpoint_work_state(bool state)
{
	setpointWorkState = state;
}

bool ClientState::get_actual_work_state() const
{
	return actualWorkState;
}

void ClientState::set_actual_work_state(bool state)
{
	actualWorkState = state;
}

bool ClientState::is_section_control_enabled() const
{
	return isSectionControlEnabled;
}

void ClientState::set_section_control_enabled(bool state)
{
	isSectionControlEnabled = state;
}

isobus::DeviceDescriptorObjectPool &ClientState::get_pool()
{
	return pool;
}

bool ClientState::are_measurement_commands_sent() const
{
	return areMeasurementCommandsSent;
}

void ClientState::mark_measurement_commands_sent()
{
	areMeasurementCommandsSent = true;
}

std::uint16_t ClientState::get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		return it->second;
	}
	std::cout << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return 0;
}

bool ClientState::try_get_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t &elementNumber) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		elementNumber = it->second;
		return true;
	}
	std::cout << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return false;
}

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

bool ClientState::get_left_tramline_state() const
{
	return leftTramlineState;
}

void ClientState::set_left_tramline_state(bool state)
{
	leftTramlineState = state;
}

bool ClientState::get_right_tramline_state() const
{
	return rightTramlineState;
}

void ClientState::set_right_tramline_state(bool state)
{
	rightTramlineState = state;
}

void ClientState::set_track_number_to_right(std::int32_t track)
{
	trackNumberToRight = track;
}

std::int32_t ClientState::get_track_number_to_right() const
{
	return trackNumberToRight;
}

void ClientState::set_track_number_to_left(std::int32_t track)
{
	trackNumberToLeft = track;
}

std::int32_t ClientState::get_track_number_to_left() const
{
	return trackNumberToLeft;
}

void ClientState::set_track_number_shift(std::int32_t shift)
{
	trackNumberShift = shift;
}

std::int32_t ClientState::get_track_number_shift() const
{
	return trackNumberShift;
}

void ClientState::set_tramline_sequence_number(std::uint32_t sequence)
{
	tramlineSequenceNumber = sequence;
}

std::uint32_t ClientState::get_tramline_sequence_number() const
{
	return tramlineSequenceNumber;
}

void ClientState::set_unique_ab_reference_id(std::uint32_t id)
{
	uniqueABReferenceID = id;
}

std::uint32_t ClientState::get_unique_ab_reference_id() const
{
	return uniqueABReferenceID;
}

void ClientState::set_tramline_control_level_support(std::uint8_t supportBits)
{
	tramlineControlLevelSupport = supportBits;
}

std::uint8_t ClientState::get_tramline_control_level_support() const
{
	return tramlineControlLevelSupport;
}

void ClientState::set_selected_tramline_control_level(std::uint8_t level)
{
	selectedTramlineControlLevel = level;
}

std::uint8_t ClientState::get_selected_tramline_control_level() const
{
	return selectedTramlineControlLevel;
}

void ClientState::set_track_number(std::uint16_t track)
{
	trackNumber = track;
}

std::uint16_t ClientState::get_track_number() const
{
	return trackNumber;
}

void ClientState::set_last_tramline_control_state_sent(std::uint8_t state)
{
	lastSentTramlineControlState = state;
}

std::uint8_t ClientState::get_last_tramline_control_state_sent() const
{
	return lastSentTramlineControlState;
}

bool ClientState::is_tramline_control_enabled() const
{
	// For now, we'll use the same approach as section control
	// In the future, we might want to store this state separately
	return (lastSentTramlineControlState == 1);
}

void ClientState::set_tramline_control_enabled(bool state)
{
	// For now, we'll use the same approach as section control
	// In the future, we might want to store this state separately
	lastSentTramlineControlState = state ? 1 : 0;
}

void ClientState::set_element_work_state(std::uint16_t elementNumber, bool isWorking)
{
	elementWorkStates[elementNumber] = isWorking;
}

bool ClientState::get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const
{
	auto it = elementWorkStates.find(elementNumber);
	if (it != elementWorkStates.end())
	{
		isWorking = it->second;
		return true;
	}
	return false;
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       16, // AOG limits to 16 sections of unique width
                       16, // 16 channels for position based control
                       isobus::TaskControllerOptions()
                         .with_implement_section_control(), // We support section control
                       TaskControllerVersion::SecondEditionDraft)
{
}

bool MyTCServer::activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &)
{
	// Safety check to make sure partnerCF has uploaded a DDOP
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		return false;
	}

	// Initialize a new client state
	auto state = ClientState();
	// state.get_pool().set_task_controller_compatibility_level(get_active_client(partnerCF)->reportedVersion);
	state.get_pool().set_task_controller_compatibility_level(static_cast<std::uint8_t>(TaskControllerVersion::SecondEditionDraft));

	bool deserialized = false;
	while (!uploadedPools[partnerCF].empty())
	{
		auto binaryPool = uploadedPools[partnerCF].front();
		uploadedPools[partnerCF].pop();
		deserialized = state.get_pool().deserialize_binary_object_pool(binaryPool.data(), static_cast<std::uint32_t>(binaryPool.size()), partnerCF->get_NAME());
	}
	if (deserialized)
	{
		std::cout << "Successfully deserialized device descriptor object pool." << std::endl;

		// Save to NVM
		std::shared_ptr<isobus::task_controller_object::DeviceObject> deviceObject;
		for (std::uint16_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::Device)
			{
				deviceObject = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
				break;
			}
		}

		auto labelBytes = deviceObject->get_localization_label();
		std::string label(reinterpret_cast<const char *>(labelBytes.data()), labelBytes.size());

		// trim at first occurrence of null or ETX (0x03)
		auto it = std::find_if(label.begin(), label.end(), [](char c) { return c == '\0' || static_cast<unsigned char>(c) == 0x03; });

		label.erase(it, label.end());

		auto fileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "\\" + label + ".ddop";

		std::vector<std::uint8_t> binaryPool;
		if (state.get_pool().generate_binary_object_pool(binaryPool))
		{
			auto fullPath = Settings::get_filename_path(fileName);
			std::cout << "Saving DDOP to: " << fullPath << std::endl;
			std::ofstream outFile(Settings::get_filename_path(fileName), std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
				outFile.close();
			}
			else
			{
				std::cout << "Unable to save DDOP to NVM. (Failed to open file)" << std::endl;
			}
		}
		else
		{
			std::cout << "Unable to save DDOP to NVM. (Failed to generate binary object pool)" << std::endl;
		}

		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(state.get_pool());
		std::uint8_t numberOfSections = 0;

		std::cout << "Implement geometry: " << std::endl;
		std::cout << "Number of booms=" << implement.booms.size() << std::endl;
		for (const auto &boom : implement.booms)
		{
			std::cout << "Boom: id=" << static_cast<int>(boom.elementNumber) << std::endl;
			for (const auto &subBoom : boom.subBooms)
			{
				std::cout << "SubBoom: id=" << static_cast<int>(subBoom.elementNumber) << std::endl;
				for (const auto &section : subBoom.sections)
				{
					numberOfSections++;

					std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
					std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
					std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
					std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
					std::cout << "Width: " << section.width_mm.get() << std::endl;
				}
			}
			for (const auto &section : boom.sections)
			{
				numberOfSections++;
				std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
				std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
				std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
				std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
				std::cout << "Width: " << section.width_mm.get() << std::endl;
			}
		}
		state.set_number_of_sections(numberOfSections);
	}
	else
	{
		std::cout << "Failed to deserialize device descriptor object pool." << std::endl;
		return false;
	}

	clients[partnerCF] = state;
	return true;
}

bool MyTCServer::change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &)
{
	return true;
}

bool MyTCServer::deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF)
{
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &)
{
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &)
{
	return false;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &)
{
	return false;
}

bool MyTCServer::get_is_enough_memory_available(std::uint32_t)
{
	return true;
}

void MyTCServer::identify_task_controller(std::uint8_t)
{
	// When this is called, the TC is supposed to display its TC number for 3 seconds if possible (which is passed into this function).
	// Your TC's number is your function code + 1, in the range of 1-32.
}

void MyTCServer::on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner)
{
	// Cleanup the client state
	clients.erase(partner);
}

void MyTCServer::on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner,
                                             std::uint16_t dataDescriptionIndex,
                                             std::uint16_t elementNumber,
                                             std::uint8_t errorCodesFromClient,
                                             ProcessDataCommands processDataCommand)
{
	// This callback lets you know when a client sends a process data acknowledge
	// (PDACK) message to you
	const auto &entry = isobus::DataDictionary::get_entry(dataDescriptionIndex);
	std::cout << "Received process data acknowledge from client "
	          << int(partner->get_address()) << " for DDI "
	          << dataDescriptionIndex << " (" << entry.to_string() << ") element "
	          << elementNumber << " with error codes "
	          << std::bitset<8>(errorCodesFromClient) << " and command "
	          << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
{
	// Human-readable DDI log for incoming value commands
	const auto &entry = isobus::DataDictionary::get_entry(dataDescriptionIndex);
	std::cout << "PD value from client " << int(partner->get_address())
	          << ": DDI " << dataDescriptionIndex << " (" << entry.to_string() << ")"
	          << ", element " << elementNumber << ", value " << processDataValue
	          << " (" << entry.format_value(processDataValue) << ")" << std::endl;

	switch (dataDescriptionIndex)
	{
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState): // Tramline Control State (DDI 515)
		{
			std::uint8_t bits = static_cast<std::uint8_t>(processDataValue) & 0x03;
			const char *mode = (bits == 0) ? "manual/off" : (bits == 1) ? "automatic/on"
			  : (bits == 2)                                             ? "error"
			                                                            : "undefined";
			std::cout << "Implement Tramline Control State: " << mode << std::endl;
			
			// Update the client state with the actual tramline control state from the implement
			bool isEnabled = (bits == 1); // automatic/on
			clients[partner].set_tramline_control_enabled(isEnabled);
		}
		break;
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel): // Tramline Control Level (DDI 505)
		{
			std::uint8_t support = static_cast<std::uint8_t>(processDataValue) & 0x07;
			clients[partner].set_tramline_control_level_support(support);
			bool l1 = (support & 0x01) != 0;
			bool l2 = (support & 0x02) != 0;
			bool l3 = (support & 0x04) != 0;
			std::cout << "Implement Tramline Control Level support: L1=" << (l1 ? "Yes" : "No")
			          << ", L2=" << (l2 ? "Yes" : "No")
			          << ", L3=" << (l3 ? "Yes" : "No") ;

			// Choose a common level with TC support. TC supports Level 1 only for now.
			constexpr std::uint8_t tcSupportedMask = 0x01; // Level 1
			std::uint8_t chosen = 0; // 0 = No common level
			if ((support & tcSupportedMask) != 0)
			{
				chosen = 1; // Use Level 1
			}

			std::cout << " Selected Tramline Control Level: " << int(chosen) << std::endl;

			// Get the current selected level before updating
			std::uint8_t currentSelected = clients[partner].get_selected_tramline_control_level();
			
			// ALWAYS update the selected tramline control level when we negotiate
			// This ensures we can track what we think we've negotiated
			clients[partner].set_selected_tramline_control_level(chosen);
			
			// Only send DDI 506 if the chosen level is different from the previously selected level
			if (chosen != currentSelected)
			{
				// Send the setpoint tramline control level to the implement
				send_setpoint_tramline_control_level(partner);
			}
			else
			{
				std::cout << "Tramline control level unchanged; skipping DDI 506 send." << std::endl;
			}
		}
		break;
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState17_32):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState33_48):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState49_64):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState65_80):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState81_96):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState97_112):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState113_128):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState129_144):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState145_160):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState161_176):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState177_192):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState193_208):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState209_224):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState225_240):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256):
		{
			std::uint8_t sectionIndexOffset = NUMBER_SECTIONS_PER_CONDENSED_MESSAGE * static_cast<std::uint8_t>(dataDescriptionIndex - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16));

			// Check if ActualWorkState is off (0) for either the current element or element 0 (main implement)
			// If either is off, all sections should be treated as off
			bool workStateOff = false;
			auto &clientState = clients[partner];

			// Check if the current element's work state is off
			bool currentElementWorkState;
			if (clientState.get_element_work_state(elementNumber, currentElementWorkState) && !currentElementWorkState)
			{
				workStateOff = true;
				std::cout << "Element " << elementNumber << " work state is OFF, forcing sections to OFF" << std::endl;
			}

			// Check if element 0's work state is off (main implement)
			bool mainElementWorkState;
			if (clientState.get_element_work_state(0, mainElementWorkState))
			{
				if (!mainElementWorkState)
				{
					workStateOff = true;
					std::cout << "Element 0 work state is OFF, forcing sections to OFF" << std::endl;
				}
			}

			for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
			{
				// When work state is off, force all sections to off state
				// Otherwise, use the actual values from the implement
				std::uint8_t sectionState = workStateOff ? SectionState::OFF : ((processDataValue >> (2 * i)) & 0x03);
				clients[partner].set_section_actual_state(i + sectionIndexOffset, sectionState);
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
		{
			clients[partner].set_section_control_enabled(processDataValue == 1);
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
		{
			// Store the work state per element rather than globally
			clients[partner].set_element_work_state(elementNumber, processDataValue == 1);
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointTramlineControlLevel): // Setpoint Tramline Control Level (DDI 506)
		{
			std::uint8_t reportedLevel = static_cast<std::uint8_t>(processDataValue & 0xFF);
			std::uint8_t currentSelected = clients[partner].get_selected_tramline_control_level();
			
			std::cout << "Implement reported Setpoint Tramline Control Level (DDI 506): " << int(reportedLevel) << std::endl;
			
			// Double-check: if the implement reports a different level than what we selected,
			// it means our negotiation was incorrect
			if (reportedLevel != currentSelected)
			{
				std::cout << "WARNING: Implement reported different tramline control level than selected!"
				          << " Selected: " << int(currentSelected) 
				          << ", Reported: " << int(reportedLevel) << std::endl;
				
				// Update our selected level to match what the implement actually accepted
				clients[partner].set_selected_tramline_control_level(reportedLevel);
			}
			else
			{
				std::cout << "Tramline control level negotiation confirmed by implement." << std::endl;
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineSequenceNumber): // Tramline Sequence Number (DDI 507)
		{
			std::int32_t reportedSequence = processDataValue;
			std::uint32_t currentSequence = clients[partner].get_tramline_sequence_number();
			
			// Handle the signed/unsigned conversion issue
			// When we send uint32_t values > INT32_MAX, they appear as negative int32_t values
			// Convert back to the proper unsigned value
			if (reportedSequence < 0)
			{
				// Convert negative int32_t back to uint32_t by adding 2^31
				reportedSequence = reportedSequence & 0x7FFFFFFF;
				std::cout << "Implement reported Tramline Sequence Number (DDI 507): " << processDataValue << " (conversion issue, using: " << reportedSequence << ")" << std::endl;
			}
			else
			{
				std::cout << "Implement reported Tramline Sequence Number (DDI 507): " << reportedSequence << std::endl;
			}
			
			// Update our local state to match what the implement reports
			if (reportedSequence != static_cast<std::int32_t>(currentSequence))
			{
				std::cout << "Tramline sequence number updated by implement: " << currentSequence 
				          << " -> " << reportedSequence << std::endl;
				clients[partner].set_tramline_sequence_number(static_cast<std::uint32_t>(reportedSequence));
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID): // Unique A-B Guidance Reference Line ID (DDI 508)
		{
			std::int32_t reportedID = processDataValue;
			std::uint32_t currentID = clients[partner].get_unique_ab_reference_id();
			
			// Handle the signed/unsigned conversion issue
			// When we send uint32_t values > INT32_MAX, they appear as negative int32_t values
			// Convert back to the proper unsigned value
			if (reportedID < 0)
			{
				// Convert negative int32_t back to uint32_t by adding 2^31
				reportedID = reportedID & 0x7FFFFFFF;
				std::cout << "Implement reported Unique A-B Guidance Reference Line ID (DDI 508): " << processDataValue << " (conversion issue, using: " << reportedID << ")" << std::endl;
			}
			else
			{
				std::cout << "Implement reported Unique A-B Guidance Reference Line ID (DDI 508): " << reportedID << std::endl;
			}
			
			// Update our local state to match what the implement reports
			if (reportedID != static_cast<std::int32_t>(currentID))
			{
				std::cout << "Unique A-B Guidance Reference Line ID updated by implement: " << currentID 
				          << " -> " << reportedID << std::endl;
				clients[partner].set_unique_ab_reference_id(static_cast<std::uint32_t>(reportedID));
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualTrackNumber): // Actual Track Number (DDI 509)
		{
			std::uint16_t reportedTrack = static_cast<std::uint16_t>(processDataValue);
			std::uint16_t currentTrack = clients[partner].get_track_number();
			
			std::cout << "Implement reported Actual Track Number (DDI 509): " << reportedTrack << std::endl;
			
			// Update our local state to match what the implement reports
			if (reportedTrack != currentTrack)
			{
				std::cout << "Actual track number updated by implement: " << currentTrack 
				          << " -> " << reportedTrack << std::endl;
				clients[partner].set_track_number(reportedTrack);
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheRight): // Track Number to the Right (DDI 510)
		{
			std::int32_t reportedTrack = processDataValue;
			std::int16_t currentTrack = clients[partner].get_track_number_to_right();
			
			// Workaround for AgIsoStack++ signed/unsigned conversion issue
			// When we send values > 10, the implement might report 0 due to conversion issues
			if (reportedTrack == 0 && currentTrack > 10)
			{
				// This is likely the conversion issue, treat as the value we sent
				reportedTrack = static_cast<std::int32_t>(currentTrack);
				std::cout << "Implement reported Track Number to the Right (DDI 510): 0 (conversion issue, using: " << reportedTrack << ")" << std::endl;
			}
			else
			{
				std::cout << "Implement reported Track Number to the Right (DDI 510): " << reportedTrack << std::endl;
			}
			
			// Update our local state to match what the implement reports
			if (reportedTrack != static_cast<std::int32_t>(currentTrack))
			{
				std::cout << "Track number to the right updated by implement: " << currentTrack 
				          << " -> " << reportedTrack << std::endl;
				clients[partner].set_track_number_to_right(static_cast<std::int16_t>(reportedTrack));
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheLeft): // Track Number to the Left (DDI 511)
		{
			std::int32_t reportedTrack = processDataValue;
			std::int16_t currentTrack = clients[partner].get_track_number_to_left();
			
			// Workaround for AgIsoStack++ signed/unsigned conversion issue
			// When we send values > 10, the implement might report 0 due to conversion issues
			if (reportedTrack == 0 && currentTrack > 10)
			{
				// This is likely the conversion issue, treat as the value we sent
				reportedTrack = static_cast<std::int32_t>(currentTrack);
				std::cout << "Implement reported Track Number to the Left (DDI 511): 0 (conversion issue, using: " << reportedTrack << ")" << std::endl;
			}
			else
			{
				std::cout << "Implement reported Track Number to the Left (DDI 511): " << reportedTrack << std::endl;
			}
			
			// Update our local state to match what the implement reports
			if (reportedTrack != static_cast<std::int32_t>(currentTrack))
			{
				std::cout << "Track number to the left updated by implement: " << currentTrack 
				          << " -> " << reportedTrack << std::endl;
				clients[partner].set_track_number_to_left(static_cast<std::int16_t>(reportedTrack));
			}
		}
		break;

		case DDI_TRACK_NUMBER_SHIFT: // Track Number Shift (DDI 669)
		{
			std::int32_t reportedShift = processDataValue;
			std::int32_t currentShift = clients[partner].get_track_number_shift();
			
			std::cout << "Implement reported Track Number Shift (DDI 669): " << reportedShift << std::endl;
			
			// Update our local state to match what the implement reports
			if (reportedShift != currentShift)
			{
				std::cout << "Track number shift updated by implement: " << currentShift 
				          << " -> " << reportedShift << std::endl;
				clients[partner].set_track_number_shift(reportedShift);
			}
		}
		break;
	}

	return true;
}

bool MyTCServer::store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool)
{
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
	}
	uploadedPools[partnerCF].push(binaryPool);
	return true;
}

std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &MyTCServer::get_clients()
{
	return clients;
}

void MyTCServer::request_measurement_commands()
{
	for (auto &client : clients)
	{
		if (!client.second.are_measurement_commands_sent())
		{
			// Find all actual (condensed) work state DDIs and request them to trigger "On Change" and "Time Interval"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the actual condensed work state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
										}
										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval))
										{
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1000);
										}
									}
								}
							}
						}
					}
				}
			}

			// Find all section control state DDIs and request them to trigger "On Change"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState) ||
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256)) ||
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState) || // Tramline Control State (DDI 515)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel) || // Tramline Control Level (DDI 505)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointTramlineControlLevel) || // Setpoint Tramline Control Level (DDI 506)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualTrackNumber) || // Actual Track Number (DDI 509)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineSequenceNumber) || // Tramline Sequence Number (DDI 507)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID) || // Unique A-B Guidance Reference Line ID (DDI 508)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheRight) || // Track Number to the Right (DDI 510)
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheLeft) || // Track Number to the Left (DDI 511)
					    processDataObject->get_ddi() == DDI_TRACK_NUMBER_SHIFT) // Track Number Shift (DDI 669)
					{
						// Loop over all objects to find the elements that are the parents of the section control state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
										std::cout << "Mapped DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
										          << elementObject->get_element_number() << std::endl;

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											          << elementObject->get_element_number() << std::endl;
										}
										if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel)) // Tramline Control Level (DDI 505)
										{
											// Ask for periodic update so we receive the supported levels at least once
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 5000);
											std::cout << "Requested Tramline Control Level (DDI 505) from element " << elementObject->get_element_number() << std::endl;
										}
										// Removed periodic updates for DDI 507 and DDI 508 as we should only use on_change
										// and properly update our side of the values when the implement sends them back
									}
								}
							}
						}
					}
				}
			}

			std::cout << "Measurement commands sent." << std::endl;
			client.second.mark_measurement_commands_sent();
		}
	}
}

void MyTCServer::update_section_states(std::vector<bool> &sectionStates)
{
	for (auto &client : clients)
	{
		auto &state = client.second;
		if (!state.is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			return;
		}

		bool requiresUpdate = false;
		for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
		{
			if ((i % NUMBER_SECTIONS_PER_CONDENSED_MESSAGE == 0) && requiresUpdate)
			{
				// Send the previous 16 sections
				std::uint8_t ddiOffset = (i / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE) - 1;
				send_section_setpoint_states(client.first, ddiOffset);
				requiresUpdate = false;
			}

			if (i < sectionStates.size())
			{
				if (sectionStates[i] != (state.get_section_setpoint_state(i) == SectionState::ON))
				{
					state.set_section_setpoint_state(i, sectionStates[i] ? SectionState::ON : SectionState::OFF);
					requiresUpdate = true;
				}
			}
		}
		if (requiresUpdate)
		{
			std::uint8_t ddiOffset = (state.get_number_of_sections() - 1) / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
			send_section_setpoint_states(client.first, ddiOffset);
		}
	}
}

void MyTCServer::update_section_control_enabled(bool enabled)
{
	for (auto &client : clients)
	{
		if (client.second.is_section_control_enabled() != enabled)
		{
			client.second.set_section_control_enabled(enabled);
			send_section_control_state(client.first, enabled);
		}
		
		// Check if we need to update the tramline control state
		// According to specification, when section control is enabled/disabled,
		// we should also check for automatic tramline control
		bool desiredTramlineState = enabled; // When section control is enabled, tramline control should also be enabled
		if(client.second.is_tramline_control_enabled() != desiredTramlineState)
		{
			std::cout << "Tramline control state needs update. Desired state: " << (enabled ? "automatic/on" : "manual/off") << std::endl;
			std::uint16_t elementNumber = 0;
			if (client.second.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::TramlineControlState, elementNumber))
			{
				// Send DDI 515 Tramline Control State to inform implement
				// 1 = automatic/on, 0 = manual/off
				std::uint8_t tramlineStateValue = desiredTramlineState ? 1 : 0;
				send_set_value(client.first, 
				               static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState), 
				               elementNumber, 
				               tramlineStateValue);
				client.second.set_tramline_control_enabled(desiredTramlineState);
				std::cout << "Sent Tramline Control State (DDI 515): " << static_cast<int>(tramlineStateValue) << std::endl;
			}
			else
			{
				std::cout << "DDI 515 element not found; unable to send Tramline Control State." << std::endl;
			}
		}
	}
}

void MyTCServer::update_tramline_states(bool leftTram, bool rightTram)
{
	for (auto &client : clients)
	{
		bool oldLeft = client.second.get_left_tramline_state();
		bool oldRight = client.second.get_right_tramline_state();
		client.second.set_left_tramline_state(leftTram);
		client.second.set_right_tramline_state(rightTram);

		// For Level 1 tramline control, we increment track number when both tramlines become active
		bool oldBothActive = oldLeft && oldRight;
		bool newBothActive = leftTram && rightTram;

		// When both tramlines become active, trigger the next tramline sequence
		if (leftTram && rightTram)
		{
			std::cout << "########      Both tramlines are now active; triggering next tramline sequence." << std::endl;
			handle_tramline_sequence(client.first);
		}
	}
}

void MyTCServer::handle_tramline_sequence(std::shared_ptr<isobus::ControlFunction> client)
{
	auto &state = clients[client];

	// Get the current track number
	std::uint16_t currentTrack = state.get_track_number();
	
	// Check if both tramlines are now active (this is when we increment the track number)
	bool leftTramActive = state.get_left_tramline_state();
	bool rightTramActive = state.get_right_tramline_state();
	if (leftTramActive && rightTramActive)
		currentTrack++;

	// For testing purposes, we'll increment the track number when both tramlines are active
	state.set_track_number(currentTrack);

	// Increment the sequence number (wrapping at 2147483647 as per specification)
	// Sequence number must start with 1 and increase on every new sequence
	std::uint32_t sequenceNumber = state.get_tramline_sequence_number() + 1;
	// Ensure sequence number starts at 1 and wraps at 2147483647 (INT32_MAX)
	if (sequenceNumber == 0 || sequenceNumber >= 2147483647)
	{
		sequenceNumber = 1;
	}
	state.set_tramline_sequence_number(sequenceNumber);

	// Hardcode the Unique A-B Guidance Reference Line ID to 1 as per specification
	std::uint32_t uniqueABReferenceID = 1;
	state.set_unique_ab_reference_id(uniqueABReferenceID);

	// Calculate track numbers to the left and right (relative to current track)
	std::int32_t trackToLeft = static_cast<std::int32_t>(currentTrack) - 1;
	std::int32_t trackToRight = static_cast<std::int32_t>(currentTrack) + 1;
	state.set_track_number_to_left(trackToLeft);
	state.set_track_number_to_right(trackToRight);

	// Send the tramline sequence data to the implement using the correct DDI values
	std::cout << "Tramline sequence triggered:" << std::endl;
	std::cout << "  Sequence Number: " << sequenceNumber << std::endl;
	std::cout << "  Unique A-B Reference ID: " << uniqueABReferenceID << std::endl;
	std::cout << "  Current Track: " << currentTrack << std::endl;
	std::cout << "  Track to Left: " << trackToLeft << std::endl;
	std::cout << "  Track to Right: " << trackToRight << std::endl;

	// According to specification, tramline parameters must be sent as a group
	// and in a specific order to ensure validity:
	// 1. DDI 507 (Tramline Sequence Number) - first
	// 2. DDI 508 (Unique A-B Guidance Reference Line ID) - immediately after DDI 507
	// 3. Other parameters (509, 510, 511, 669) - after DDI 508

	// Send Tramline Sequence Number (DDI 507) - MUST be first
	std::uint16_t elem507 = 0;
	if (state.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::TramlineSequenceNumber, elem507))
	{
		// For Tramline Sequence Number (DDI 507), use uint32_t as per specification (0 to 2147483647)
		// Send as uint32_t directly without casting to int32_t to avoid conversion issues
		std::int32_t valueToSend = static_cast<std::int32_t>(sequenceNumber & 0x7FFFFFFF);
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineSequenceNumber), elem507, valueToSend);
		std::cout << "Sent Tramline Sequence Number (DDI 507): " << sequenceNumber << " (raw: " << valueToSend << ")" << std::endl;
	}
	else
	{
		std::cout << "DDI 507 element not found in DDOP; unable to send Tramline Sequence Number." << std::endl;
	}

	// Send Unique A-B Guidance Reference Line ID (DDI 508) - MUST be immediately after DDI 507
	std::uint16_t elem508 = 0;
	if (state.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID, elem508))
	{
		// For Unique A-B Guidance Reference Line ID (DDI 508), use uint32_t as per specification (0 to 2147483647)
		// Send as uint32_t directly without casting to int32_t to avoid conversion issues
		std::int32_t valueToSend = static_cast<std::int32_t>(uniqueABReferenceID & 0x7FFFFFFF);
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID), elem508, valueToSend);
		std::cout << "Sent Unique A-B Guidance Reference Line ID (DDI 508): " << uniqueABReferenceID << " (raw: " << valueToSend << ")" << std::endl;
	}
	else
	{
		std::cout << "DDI 508 element not found in DDOP; unable to send Unique A-B Guidance Reference Line ID." << std::endl;
	}

	// Send Actual Track Number (DDI 509)
	std::uint16_t elem509 = 0;
	if (state.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::ActualTrackNumber, elem509))
	{
		// For Actual Track Number (DDI 509), use int32_t as per specification (-2147483648 to 2147483647)
		std::int32_t valueToSend = static_cast<std::int32_t>(currentTrack);
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualTrackNumber), elem509, valueToSend);
		std::cout << "Sent Actual Track Number (DDI 509): " << valueToSend << std::endl;
	}
	else
	{
		std::cout << "DDI 509 element not found in DDOP; unable to send Actual Track Number." << std::endl;
	}

	// Send Track Number to the Right (DDI 510)
	std::uint16_t elem510 = 0;
	if (state.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::TrackNumberToTheRight, elem510))
	{
		// For Track Number to the Right (DDI 510), use int32_t as per specification (-2147483648 to 2147483647)
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheRight), elem510, trackToRight);
		std::cout << "Sent Track Number to the Right (DDI 510): " << trackToRight << std::endl;
	}
	else
	{
		std::cout << "DDI 510 element not found in DDOP; unable to send Track Number to the Right." << std::endl;
	}

	// Send Track Number to the Left (DDI 511)
	std::uint16_t elem511 = 0;
	if (state.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::TrackNumberToTheLeft, elem511))
	{
		// For Track Number to the Left (DDI 511), use int32_t as per specification (-2147483648 to 2147483647)
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheLeft), elem511, trackToLeft);
		std::cout << "Sent Track Number to the Left (DDI 511): " << trackToLeft << std::endl;
	}
	else
	{
		std::cout << "DDI 511 element not found in DDOP; unable to send Track Number to the Left." << std::endl;
	}

	// Send Track Number Shift (DDI 669) - should be sent after the other parameters as per specification
	std::uint16_t elem669 = 0;
	if (state.try_get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(DDI_TRACK_NUMBER_SHIFT), elem669))
	{
		// For Track Number Shift (DDI 669), use int32_t as per specification (-2147483648 to 2147483647)
		std::int32_t trackNumberShift = state.get_track_number_shift();
		send_set_value(client, DDI_TRACK_NUMBER_SHIFT, elem669, trackNumberShift);
		std::cout << "Sent Track Number Shift (DDI 669): " << trackNumberShift << std::endl;
	}
	else
	{
		std::cout << "DDI 669 element not found in DDOP; unable to send Track Number Shift." << std::endl;
	}

}

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
{
    // Modern ECU? (DDI 290 exists)
    std::uint16_t elementNumber = 0;
    if (clients[client].try_get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16, elementNumber))
    {
        // --- MODERN TC-SC PATH (what you already had) ---
		std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
		std::uint32_t value = 0;
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
		}

		std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
		send_set_value(client, ddiTarget, clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)), value);

	}
	else if (clients[client].try_get_element_number_for_ddi(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16, elementNumber))
    {
        // --- LEGACY PATH ---
		std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
		std::uint32_t value = 0;
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
		}

		std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) + ddiOffset;
		send_set_value(client, ddiTarget, clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)), value);
    }
    else
    {
		std::cout << "[TC Server] Neither condensed nor controllable-actual work state supported Missing DDI 290 and 141!" << std::endl;
        return;
    }

	bool setpointWorkState = clients[client].is_any_section_setpoint_on();
	if ((clients[client].get_setpoint_work_state() != setpointWorkState))
	{
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
		clients[client].set_setpoint_work_state(setpointWorkState);
	}
}

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}

void MyTCServer::set_left_tramline_state(bool state)
{
	for (auto &client : clients)
	{
		client.second.set_left_tramline_state(state);
	}
}

void MyTCServer::set_right_tramline_state(bool state)
{
	for (auto &client : clients)
	{
		client.second.set_right_tramline_state(state);
	}
}

void MyTCServer::send_setpoint_tramline_control_level(std::shared_ptr<isobus::ControlFunction> client)
{
	auto &clientState = clients[client];
	
	// Get the selected tramline control level
	std::uint8_t selectedLevel = clientState.get_selected_tramline_control_level();
	
	// Only send DDI 506 if we have a valid selected level
	if (selectedLevel != 0)
	{
		// Send DDI 506 Setpoint Tramline Control Level to inform implement
		std::uint16_t elem = 0;
		if (clientState.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointTramlineControlLevel, elem))
		{
			send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointTramlineControlLevel), elem, selectedLevel);
			std::cout << "Setpoint Tramline Control Level (DDI 506) sent: " << int(selectedLevel) << " (element " << elem << ") -----------" << std::endl;
		}
		else
		{
			// Log that we can't send DDI 506, but this is not necessarily an error
			// Some implements may not include DDI 506 in their DDOP, which is non-compliant but happens in practice
			std::cout << "DDI 506 element not found in DDOP; unable to send Setpoint Tramline Control Level." << std::endl;
		}
	}
	else
	{
		std::cout << "No selected tramline control level; skipping DDI 506 send." << std::endl;
	}
}

void MyTCServer::set_track_number_shift(std::shared_ptr<isobus::ControlFunction> client, std::int32_t shift)
{
	// Set the track number shift for the client
	clients[client].set_track_number_shift(shift);
	
	// Send the track number shift to the implement if the element exists
	std::uint16_t elem = 0;
	if (clients[client].try_get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(DDI_TRACK_NUMBER_SHIFT), elem))
	{
		send_set_value(client, DDI_TRACK_NUMBER_SHIFT, elem, shift);
		std::cout << "Sent Track Number Shift (DDI 669): " << shift << std::endl;
	}
	else
	{
		std::cout << "DDI 669 element not found in DDOP; unable to send Track Number Shift." << std::endl;
	}
}
