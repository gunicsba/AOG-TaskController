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
#include "isobus/utility/system_timing.hpp"

#include <bitset>
#include <fstream>
#include <iostream>
#include <set>

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

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

bool ClientState::has_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	return ddiToElementNumber.find(ddi) != ddiToElementNumber.end();
}

const std::map<isobus::DataDescriptionIndex, std::uint16_t> &ClientState::get_ddi_to_element_map() const
{
	return ddiToElementNumber;
}

void ClientState::set_element_work_state(std::uint16_t elementNumber, bool isWorking)
{
	elementWorkStates[elementNumber] = isWorking;
}

bool ClientState::try_get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const
{
	auto it = elementWorkStates.find(elementNumber);
	if (it != elementWorkStates.end())
	{
		isWorking = it->second;
		return true;
	}
	return false;
}

void ClientState::set_ddi_value(isobus::DataDescriptionIndex ddi, std::int32_t value)
{
	ddiValues[ddi] = value;
}

bool ClientState::try_get_ddi_value(isobus::DataDescriptionIndex ddi, std::int32_t &value) const
{
	auto it = ddiValues.find(ddi);
	if (it != ddiValues.end())
	{
		value = it->second;
		return true;
	}
	return false;
}

void ClientState::set_ddi_element_value(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber, std::int32_t value)
{
	ddiElementValues[std::make_pair(ddi, elementNumber)] = value;
}

bool ClientState::try_get_ddi_element_value(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber, std::int32_t &value) const
{
	auto it = ddiElementValues.find(std::make_pair(ddi, elementNumber));
	if (it != ddiElementValues.end())
	{
		value = it->second;
		return true;
	}
	return false;
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       64, // AOG limits to 16 sections of unique width but with zones it can be 64
                       64, // 64 channels for position based control
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
			std::ofstream outFile(Settings::get_filename_path(fileName), std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
				outFile.close();
				std::cout << "Saved DDOP to file: " << fileName << std::endl;
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
		std::uint8_t numberOfSections = print_implement_geometry(implement);
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
	// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
	std::cout << "Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
{
	// Store the value for later retrieval
	auto ddi = static_cast<isobus::DataDescriptionIndex>(dataDescriptionIndex);
	clients[partner].set_ddi_value(ddi, processDataValue);
	clients[partner].set_ddi_element_value(ddi, elementNumber, processDataValue);
	
	switch (dataDescriptionIndex)
	{
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
			if (clientState.try_get_element_work_state(elementNumber, currentElementWorkState) && !currentElementWorkState)
			{
				workStateOff = true;
				//std::cout << "Element " << elementNumber << " work state is OFF, forcing sections to OFF" << std::endl;
			}
			// Check if element 0's work state is off (main implement)
			bool mainElementWorkState;
			if (clientState.try_get_element_work_state(0, mainElementWorkState))
			{
				if (!mainElementWorkState)
				{
					workStateOff = true;
					//std::cout << "Element 0 work state is OFF, forcing sections to OFF" << std::endl;
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
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
										std::cout << "Mapped DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
										          << elementObject->get_element_number() << std::endl;

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											          << elementObject->get_element_number() << std::endl;
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
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256)))
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

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											          << elementObject->get_element_number() << std::endl;
										}
										else
										{
											std::cout << "Mapped (no OnChange) DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
											          << elementObject->get_element_number() << std::endl;
										}
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

std::uint8_t MyTCServer::print_implement_geometry(const auto &implement)
{
	std::uint8_t numberOfSections = 0;

	std::cout << "Implement geometry: " << std::endl;
	std::cout << "Number of booms=" << implement.booms.size() << std::endl;
	for (const auto &boom : implement.booms)
	{
		std::cout << " Boom: id=" << static_cast<int>(boom.elementNumber) << std::endl;
		std::cout << "  X Offset: " << boom.xOffset_mm.get() << std::endl;
		std::cout << "  Y Offset: " << boom.yOffset_mm.get() << std::endl;
		std::cout << "  Z Offset: " << boom.zOffset_mm.get() << std::endl;
		for (const auto &subBoom : boom.subBooms)
		{
			std::cout << "   SubBoom: id=" << static_cast<int>(subBoom.elementNumber) << std::endl;
			std::cout << "    X Offset: " << subBoom.xOffset_mm.get() << std::endl;
			std::cout << "    Y Offset: " << subBoom.yOffset_mm.get() << std::endl;
			std::cout << "    Z Offset: " << subBoom.zOffset_mm.get() << std::endl;
			for (const auto &section : subBoom.sections)
			{
				numberOfSections++;
				std::cout << "     Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
				std::cout << "      X Offset: " << section.xOffset_mm.get() << std::endl;
				std::cout << "      Y Offset: " << section.yOffset_mm.get() << std::endl;
				std::cout << "      Z Offset: " << section.zOffset_mm.get() << std::endl;
				std::cout << "      Width: " << section.width_mm.get() << std::endl;
			}
		}
		for (const auto &section : boom.sections)
		{
			numberOfSections++;
			std::cout << "  Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
			std::cout << "   X Offset: " << section.xOffset_mm.get() << std::endl;
			std::cout << "   Y Offset: " << section.yOffset_mm.get() << std::endl;
			std::cout << "   Z Offset: " << section.zOffset_mm.get() << std::endl;
			std::cout << "   Width: " << section.width_mm.get() << std::endl;
		}
	}

	return numberOfSections;
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
	}
}

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
{
	std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
	std::uint32_t value = 0;
	for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
	{
		value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
	}

	// Modern ECU? (DDI 290  SetpointCondensedWorkState1_16 exists)
	std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
	if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)))
	{
		std::uint16_t elementNumber = clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget));
		send_set_value(client, ddiTarget, elementNumber, value);

		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if ((clients[client].get_setpoint_work_state() != setpointWorkState))
		{
			send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
			clients[client].set_setpoint_work_state(setpointWorkState);
		}
		return;
	}
	ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) + ddiOffset;
	if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)))
	{
		send_set_value(client, ddiTarget, clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)), value);
		return;
	}

	std::cout << "[TC Server] Neither condensed nor controllable-actual work state supported Missing DDI 290 and 141!" << std::endl;
}

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}

bool MyTCServer::generate_element_data_dump(std::shared_ptr<isobus::ControlFunction> client, std::string &output)
{
	try
	{
		output.clear();
		
		// Header
		output += "Element#,DDI#,Value\n";
		
		auto &clientState = clients.at(client);
		auto &pool = clientState.get_pool();
		
		// Collect all unique (DDI, Element) pairs we have values for
		std::set<std::pair<isobus::DataDescriptionIndex, std::uint16_t>> outputtedPairs;
		
		// Iterate through all elements we know about
		std::set<std::uint16_t> knownElements;
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			bool found = false;
			for (std::uint32_t j = 0; j < pool.size(); j++)
			{
				auto object = pool.get_object_by_index(j);
				if (object == nullptr)
					continue;
				
				if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
				{
					auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
					if (elementObject != nullptr && elementObject->get_element_number() == i)
					{
						knownElements.insert(i);
						found = true;
						break;
					}
				}
			}
			if (!found && i > 20)  // If no element found after 20 and none recently, we're done
				break;
		}
		knownElements.insert(0); // Add element 0
		
		// Iterate through all Device Process Data objects in the pool
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object == nullptr)
				continue;
			
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
			{
				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				if (processDataObject == nullptr)
					continue;
				
				auto ddi = static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi());
				
				// For each known element, output value if we have it (or empty if we don't)
				for (std::uint16_t elementNum : knownElements)
				{
					auto pair = std::make_pair(ddi, elementNum);
					// Skip if we've already output this pair
					if (outputtedPairs.count(pair) > 0)
						continue;
					
					std::int32_t value = 0;
					if (clientState.try_get_ddi_element_value(ddi, elementNum, value))
					{
						output += std::to_string(elementNum) + "," + std::to_string(static_cast<std::uint16_t>(ddi)) + "," + std::to_string(value) + "\n";
					}
					else
					{
						// No value yet, output with empty value
						output += std::to_string(elementNum) + "," + std::to_string(static_cast<std::uint16_t>(ddi)) + "," + "\n";
					}
					outputtedPairs.insert(pair);
				}
			}
		}
		
		return true;
	}
	catch (...)
	{
		return false;
	}
}

void MyTCServer::save_ddop_with_values_periodic()
{
	// Save DDOP and CSV with collected values every 30 seconds for each client
	if (!isobus::SystemTiming::time_expired_ms(lastDdopWithValuesUpdate, 30000))
	{
		return; // Not yet time to update
	}

	lastDdopWithValuesUpdate = isobus::SystemTiming::get_timestamp_ms();

	for (auto &client : clients)
	{
		auto partnerCF = client.first;
		
		// Save DDOP with values
		std::vector<std::uint8_t> updatedDDOP;
		if (regenerate_ddop_with_values(partnerCF, updatedDDOP))
		{
			auto ddopFileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "\\with_values.ddop";
			std::ofstream ddopFile(Settings::get_filename_path(ddopFileName), std::ios::binary);
			if (ddopFile.is_open())
			{
				ddopFile.write(reinterpret_cast<const char *>(updatedDDOP.data()), updatedDDOP.size());
				ddopFile.close();
				std::cout << "Saved DDOP with values to: " << ddopFileName << std::endl;
			}
			else
			{
				std::cout << "Failed to open file for DDOP with values: " << ddopFileName << std::endl;
			}
		}
		else
		{
			std::cout << "Failed to regenerate DDOP with values" << std::endl;
		}
		
		// Save CSV with element data
		std::string csvContent;
		if (generate_element_data_dump(partnerCF, csvContent))
		{
			auto csvFileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "\\element_data.csv";
			std::ofstream csvFile(Settings::get_filename_path(csvFileName));
			if (csvFile.is_open())
			{
				csvFile.write(csvContent.c_str(), csvContent.size());
				csvFile.close();
				std::cout << "Saved element data CSV to: " << csvFileName << std::endl;
			}
			else
			{
				std::cout << "Failed to open file for element data CSV: " << csvFileName << std::endl;
			}
		}
		else
		{
			std::cout << "Failed to generate element data CSV" << std::endl;
		}
	}
}

bool MyTCServer::regenerate_ddop_with_values(std::shared_ptr<isobus::ControlFunction> client, std::vector<std::uint8_t> &updatedBinaryDDOP)
{
	try
	{
		auto &clientState = clients.at(client);
		
		// Create a working copy of the pool
		auto workingPool = clientState.get_pool();
		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(clientState.get_pool());
		std::uint8_t numberOfSections = print_implement_geometry(implement);
		
		// Iterate through all objects in the pool to find Device Property Objects
		for (std::uint32_t i = 0; i < workingPool.size(); i++)
		{
			auto object = workingPool.get_object_by_index(i);
			if (object == nullptr)
				continue;
			
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProperty)
			{
				auto propertyObject = std::dynamic_pointer_cast<isobus::task_controller_object::DevicePropertyObject>(object);
				if (propertyObject == nullptr)
					continue;
				
				auto ddi = static_cast<isobus::DataDescriptionIndex>(propertyObject->get_ddi());
				
				// Check if we have a value for this DDI
				std::int32_t value = 0;
				if (clientState.try_get_ddi_value(ddi, value))
				{
					// Update the Device Property Object with the collected value
					propertyObject->set_value(value);
					std::cout << "Injected value " << value << " into Device Property DDI " << static_cast<std::uint16_t>(ddi) << std::endl;
				}
				else
				{
					std::cout << "Warning: No value received for Device Property DDI " << static_cast<std::uint16_t>(ddi) << std::endl;
				}
			}
		}
		
		// Generate binary DDOP with updated values
		if (workingPool.generate_binary_object_pool(updatedBinaryDDOP))
		{
			std::cout << "Successfully regenerated DDOP with collected values" << std::endl;
			return true;
		}
		else
		{
			std::cout << "Failed to generate binary DDOP" << std::endl;
			return false;
		}
	}
	catch (const std::exception &e)
	{
		std::cout << "Exception while regenerating DDOP: " << e.what() << std::endl;
		return false;
	}
}

void MyTCServer::refresh_implement_geometry()
{
	for (auto &client : clients)
	{
		auto partnerCF = client.first;
		auto &clientState = client.second;
		auto &pool = clientState.get_pool();

		// After getting your implement geometry
		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool);

		// Check each section to see if offsets need to be queried
		for (auto& boom : implement.booms) {
			for (auto& section : boom.sections) {
				// Check if offset values are NOT present in DDOP
				// If they don't exist, we need to request them from the implement
				if (!section.xOffset_mm.exists()) {
					// X offset was not defined in DDOP - need to request from implement
					send_request_value(partnerCF, 
											static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
											section.elementNumber);
				}
				if (!section.yOffset_mm.exists()) {
					// Y offset was not defined in DDOP - need to request from implement
					send_request_value(partnerCF, 
											static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
											section.elementNumber);
				}
				if (!section.zOffset_mm.exists()) {
					// Z offset was not defined in DDOP - need to request from implement
					send_request_value(partnerCF, 
											static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetZ),
											section.elementNumber);
				}
				
				// Width check (note: use get_width_with_priority() which handles priority logic)
				if (!section.width_mm.exists()) {
					// No width defined in any form - request actual working width
					send_request_value(partnerCF, 
											static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
											section.elementNumber);
				}
			}
		}
	}
	// Then, when the implement responds with values via process data commands,
	// your on_value_command callback will be called with the actual values
}

void MyTCServer::request_all_ddi_values_periodic()
{
	// Request all DDI values from all connected clients every 30 seconds
	if (!isobus::SystemTiming::time_expired_ms(lastDdiValuesRequest, 20000))
	{
		return; // Not yet time to request
	}
	refresh_implement_geometry();

	lastDdiValuesRequest = isobus::SystemTiming::get_timestamp_ms();

	for (auto &client : clients)
	{
		auto partnerCF = client.first;
		auto &clientState = client.second;
		auto &pool = clientState.get_pool();
		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(clientState.get_pool());
		
		// Build a map of element number -> list of DDIs that belong to that element
		std::map<std::uint16_t, std::set<isobus::DataDescriptionIndex>> elementToDDIs;
		
		// First pass: collect all element numbers from Device Elements
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object == nullptr)
				continue;
			
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
			{
				auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
				if (elementObject != nullptr)
				{
					std::uint16_t elementNum = elementObject->get_element_number();
					// Initialize element in map (even if it has no DDIs yet)
					elementToDDIs[elementNum];
				}
			}
		}
		
		// Also ensure element 0 is in the map
		elementToDDIs[0];
		
		// Second pass: assign DDIs to elements
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object == nullptr)
				continue;
			
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
			{
				auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
				if (elementObject == nullptr)
					continue;
				
				std::uint16_t elementNum = elementObject->get_element_number();
				
				// For each child object ID of this element
				for (std::uint16_t childId : elementObject->get_child_object_ids())
				{
					// Find the object with this ID
					for (std::uint32_t j = 0; j < pool.size(); j++)
					{
						auto childObject = pool.get_object_by_index(j);
						if (childObject == nullptr)
							continue;
						
						if (childObject->get_object_id() == childId &&
						    childObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
						{
							auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(childObject);
							if (processDataObject != nullptr)
							{
								auto ddi = static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi());
								elementToDDIs[elementNum].insert(ddi);
							}
						}
					}
				}
			}
		}
		
		// Third pass: add top-level DDIs (not assigned to specific elements) to element 0
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object == nullptr)
				continue;
			
			// Add all DeviceProcessData objects (not children of specific elements) to element 0
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
			{
				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				if (processDataObject != nullptr)
				{
					// Check if this DDI is already mapped to a specific element
					bool isTopLevel = true;
					for (const auto &elementDDIPair : elementToDDIs)
					{
						if (elementDDIPair.first != 0 && elementDDIPair.second.count(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi())) > 0)
						{
							isTopLevel = false;
							break;
						}
					}
					
					if (isTopLevel)
					{
						auto ddi = static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi());
						elementToDDIs[0].insert(ddi);
					}
				}
			}
		}
		
		// Now request values only for DDIs that belong to each element
		for (const auto &elementDDIPair : elementToDDIs)
		{
			std::uint16_t elementNum = elementDDIPair.first;
			const auto &ddiSet = elementDDIPair.second;
			
			for (isobus::DataDescriptionIndex ddi : ddiSet)
			{
				send_request_value(partnerCF, static_cast<std::uint16_t>(ddi), elementNum);
			}
		}
	}
}

