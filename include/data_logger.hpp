/**
 * @author Daan Steenbergen
 * @brief Data logging module for ISOBUS Task Controller
 * @version 0.1
 * @date 2025-4-6
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include "settings.hpp"
#include "task_controller.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

/// @brief GPS/IMU data structure for logging
struct GpsData
{
	double latitude = 0.0;
	double longitude = 0.0;
	double height = 0.0;
	double speed = 0.0;
	std::uint8_t dgpsState = 0;
	std::uint8_t dgpsAge = 0;
	double roll = 0.0;
	float headingDual = 0.0f;
	float headingTrue = 0.0f;
	std::uint16_t satellites = 0;
	std::uint16_t hdopX100 = 0;
	float imuHeading = 0.0f;
	std::int16_t imuRoll = 0;
	std::int16_t imuPitch = 0;
	std::uint16_t imuYaw = 0;
	bool isValid = false;
};

/// @brief DDI value storage with element number
struct DdiValue
{
	std::uint16_t elementNumber = 0;
	std::uint16_t ddi = 0;
	std::int32_t value = 0;
	std::string prettyName;
	bool isValid = false;
};

/// @brief Client data snapshot for logging
struct ClientDataSnapshot
{
	std::string clientName;
	std::uint8_t numberOfSections = 0;
	std::vector<std::uint8_t> sectionStates;
	bool workState = false;
	bool sectionControlEnabled = false;
	std::map<std::uint16_t, DdiValue> ddiValues; // Key: DDI number
};

/// @brief Manages CSV file writing with rotation
class CsvWriter
{
public:
	CsvWriter(const std::string &basePath, std::uint32_t maxFileSizeMB);
	~CsvWriter();

	/// @brief Open a new CSV file with the given headers
	bool open(const std::vector<std::string> &headers);

	/// @brief Close the current CSV file
	void close();

	/// @brief Write a data row to the CSV
	bool writeRow(const std::vector<std::string> &values);

	/// @brief Check if file needs rotation
	bool needsRotation() const;

	/// @brief Rotate to a new file
	bool rotate();

	/// @brief Get the current file path
	std::string getCurrentFilePath() const;

	/// @brief Set the CSV delimiter
	/// @param delimiter The delimiter to use
	void setDelimiter(const std::string &delimiter);

private:
	std::string generateFileName() const;
	std::string generateTimestamp() const;

	std::string basePath;
	std::uint32_t maxFileSizeBytes;
	std::ofstream currentFile;
	std::string currentFilePath;
	std::vector<std::string> columnHeaders;
	int rotationCounter = 0;
	std::string delimiter = ";";
	mutable std::mutex fileMutex;
};

/// @brief Manages archival of old data files
class ArchiveManager
{
public:
	/// @brief Archive old files from the specified folder
	/// @param folderPath Path to the folder to archive
	/// @param archiveFolderPath Path to store the archive
	/// @return True if successful, false otherwise
	static bool archiveFolder(const std::string &folderPath, const std::string &archiveFolderPath);

private:
	static std::string generateArchiveName();
};

/// @brief Main data logging coordinator
class DataLogger
{
public:
	DataLogger(std::shared_ptr<Settings> settings);
	~DataLogger();

	/// @brief Initialize the data logger
	/// @return True if successful, false otherwise
	bool initialize();

	/// @brief Shutdown the data logger
	void shutdown();

	/// @brief Log a data frame with current GPS and ISOBUS data
	/// @param gpsData Current GPS data
	/// @param clients Map of ISOBUS clients and their states
	void logDataFrame(const GpsData &gpsData, const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients);

	/// @brief Update DDI value for a specific client
	/// @param client The ISOBUS client
	/// @param elementNumber The element number
	/// @param ddi The DDI number
	/// @param value The process data value
	/// @param prettyName The pretty name for the DDI
	void updateDdiValue(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t value, const std::string &prettyName);

	/// @brief Check if data logging is enabled
	bool isEnabled() const;

	/// @brief Check if blockage monitoring is enabled
	bool isBlockageMonitoringEnabled() const;

private:
	void archiveOldFiles();
	std::string getDataFolderPath() const;
	std::string getArchiveFolderPath() const;
	std::string getLogsFolderPath() const;
	std::vector<std::string> buildHeaders(const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients);
	std::vector<std::string> buildRow(const GpsData &gpsData, const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients);
	std::string getDdiColumnName(const DdiValue &ddiValue) const;
	std::string formatTimestamp(const std::chrono::system_clock::time_point &time) const;

	std::shared_ptr<Settings> settings;
	std::unique_ptr<CsvWriter> csvWriter;

	// DDI value cache: client -> (elementNumber_ddi -> DdiValue)
	std::map<std::string, std::map<std::string, DdiValue>> ddiCache;
	std::mutex ddiCacheMutex;

	// Rate limiting
	std::chrono::steady_clock::time_point lastLogTime;
	std::uint32_t minLogIntervalMs = 100; // 10Hz default

	bool initialized = false;
};
