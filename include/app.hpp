/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_functionalities.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/isobus_time_date_interface.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "field_registry.hpp"
#include "guidance_track_context.hpp"
#include "logging_utils.hpp"
#include "settings.hpp"
#include "task_controller.hpp"
#include "tractor_facilities.hpp"
#include "udp_connections.hpp"

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	struct ImplementDetails
	{
		std::string displayName = "No implement";
		std::uint8_t sections = 0;
		std::string widthText;
		std::string boomOffsetText;
	};

	struct ImplementSnapshot
	{
		std::string displayName;
		std::uint8_t sections = 0;
		std::string widthText;
	};

	void send_task_controller_status_message();

	bool setup_can_hardware();
	bool setup_control_functions();
	void setup_task_controller_server();
	void setup_tecu_interfaces();
	void setup_udp_connections();

	void setup_vt_client();
	void update_vt_client();
	void try_start_vt_client();
	void nudge_offline_vt();
	void handle_vt_disconnected();
	void log_vt_capabilities_once();
	void sync_vt_config_once();
	void update_vt_section_map();
	void update_vt_status_strings(bool aogConnected);
	void update_hydration_snapshot();

	void send_vt_string_if_changed(std::uint16_t objectID, const std::string &value);
	void send_hardware_message(const std::string &text, std::uint8_t duration, std::uint8_t color);
	ImplementDetails derive_implement_details(ClientState &state) const;

	static constexpr std::uint8_t HW_MSG_ALERT = 0;
	static constexpr std::uint8_t HW_MSG_INFO = 1;

	bool is_aog_connected() const;

	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::InternalControlFunction> tcCF = nullptr;
	std::shared_ptr<isobus::InternalControlFunction> tecuCF = nullptr;
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::unique_ptr<TractorFacilities> tractorFacilities;
	std::unique_ptr<isobus::TimeDateInterface> timeDateInterface;
	std::uint32_t lastFee6TransmitMs = 0; ///< Timestamp of last FEE6 transmission
	std::uint32_t lastExternalFee6Ms = 0; ///< Timestamp of last FEE6 from another ECU (0 = never)
	bool fee6Broadcasting = false; ///< Whether we are actively broadcasting FEE6
	/// Guards lastExternalFee6Ms/fee6Broadcasting: TimeDateInterface's event listener
	/// (registered in setup_tecu_interfaces()) fires from the isobus stack's background
	/// thread — not deferred like MyTCServer's callbacks — concurrently with
	/// Application::update()'s FEE6 broadcast logic on the main thread. See
	/// docs/CONCURRENCY.md.
	std::mutex fee6Mutex;
	static constexpr std::uint32_t FEE6_TX_INTERVAL_MS = 10000; ///< FEE6 broadcast interval (10 s)
	static constexpr std::uint32_t FEE6_PROVIDER_TIMEOUT_MS = 30000; ///< If no FEE6 from other ECU for 30 s, assume no provider
	std::unique_ptr<isobus::ControlFunctionFunctionalities> tecuFunctionalities;
	std::unique_ptr<isobus::ControlFunctionFunctionalities> tcFunctionalities;
	std::shared_ptr<isobus::VirtualTerminalClient> vtClient;
	std::unique_ptr<isobus::VirtualTerminalClientUpdateHelper> vtUpdateHelper;
	std::vector<std::uint8_t> vtObjectPool;
	bool vtClientStarted = false;
	/// The isobus network manager marks external control functions offline 755 ms after a global address
	/// claim request unless they claimed since; 250 ms of margin covers the delay between creating an
	/// internal control function and its request reaching the bus.
	static constexpr std::uint32_t ADDRESS_CLAIM_SETTLE_MS = 1250;
	std::uint32_t lastInternalCfCreatedMs = 0; ///< When the last internal control function (whose creation sends an address claim request) was created
	static constexpr std::uint32_t VT_NUDGE_INTERVAL_MS = 3000; ///< Spacing between address claim requests while the VT is offline
	static constexpr std::uint8_t VT_NUDGE_MAX_ATTEMPTS = 10; ///< Give up (until the VT is seen again) after this many requests
	std::uint32_t vtNudgeLastMs = 0; ///< When the VT was last seen offline / last nudged, 0 while it is online
	std::uint8_t vtNudgeCount = 0;
	bool canHardwareStarted = false; ///< CANHardwareInterface::start() succeeded, so stop() has a thread to stop
	bool vtConfigSynced = false;
	bool vtWasConnected = false;
	bool vtConnectionWarningLogged = false;
	bool vtCapabilitiesLogged = false;
	std::atomic<bool> hydrationSnapshotRequested{ false }; ///< Set by the VT button listener, handled in update()
	std::uint8_t nmea2000SequenceIdentifier = 0;
	std::uint32_t lastJ1939SpeedTransmit = 0;
	std::uint32_t lastTCStatusTransmit = 0;
	static constexpr std::uint32_t TC_STATUS_STARTUP_DELAY_MS = 6000; ///< ISO 11783-10 6.6.1: TC waits 6 s after its address claim before sending TC Status
	std::uint32_t tcAddressClaimedMs = 0; ///< When the TC's address claim completed, 0 = not yet
	std::int32_t lastSpeedValue = 0;
	std::int32_t lastXteValue = 0;
	std::uint8_t gnssFixQuality = 0; ///< AOG fix quality (NMEA 2000 GNSS Method): 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK Fix, 5=Float, 6=Estimated, 7=Manual, 8=Simulated
	std::uint32_t lastGnssQualityMs = 0; ///< Timestamp of last PGN 0xD6 fix-quality update (0 = never received)
	static constexpr std::uint32_t GNSS_QUALITY_TIMEOUT_MS = 2000; ///< No PGN 0xD6 for this long = fix quality unknown
	static constexpr std::uint32_t AOG_CONNECTION_TIMEOUT_MS = 3000; ///< No AOG packet for this long = disconnected

	// Guidance track context — real data from AOG PGN 0xF4.
	GuidanceTrackProvider trackProvider;
	GuidanceTrackContext currentTrackContext;
	bool aogWasConnectedForTrack = false; ///< Edge-detection for AOG connect/disconnect transitions

	// Field identity — from AOG PGN 0xF3. Folded into the upper 16 bits of DDI 508
	// (see the PGN 0xF4 handling in setup_udp_connections()) so a track's guidance reference
	// line ID is unique across fields, not just within whichever field AOG currently has open.
	FieldRegistry fieldRegistry;
	std::string currentFieldName; ///< Empty when no field is open
	std::uint16_t currentFieldIndex = 0;
	bool hasActiveField = false;

	std::uint32_t lastDistanceMm = 0;
	std::uint32_t lastAogPacketMs = 0;

	bool tractorFacilitiesSentOnPowerUp = false;
	std::uint32_t vtDisconnectedSinceMs = 0;
	std::uint32_t lastVtStatusUpdateMs = 0;
	std::uint32_t lastVtSectionUpdateMs = 0;
	bool tcAddressConflictActive = false;
	bool tecuAddressClaimFailed = false;
	std::map<std::uint64_t, ImplementSnapshot> implementSnapshots;
	std::map<std::uint16_t, std::string> lastVtStrings;
};
