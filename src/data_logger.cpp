/**
 * @author Daan Steenbergen
 * @brief Data logging module for ISOBUS Task Controller
 * @version 0.1
 * @date 2025-4-6
 *
 * @copyright 2025 Daan Steenbergen
 */

#include "data_logger.hpp"
#include "logging_utils.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#endif

// CsvWriter Implementation

CsvWriter::CsvWriter(const std::string &basePath, std::uint32_t maxFileSizeMB) :
  basePath(basePath),
  maxFileSizeBytes(maxFileSizeMB * 1024 * 1024)
{
}

CsvWriter::~CsvWriter()
{
	close();
}

bool CsvWriter::open(const std::vector<std::string> &headers)
{
	std::lock_guard<std::mutex> lock(fileMutex);

	columnHeaders = headers;
	currentFilePath = generateFileName();

	// Ensure directory exists
	std::filesystem::path dirPath = std::filesystem::path(currentFilePath).parent_path();
	if (!std::filesystem::exists(dirPath))
	{
		std::filesystem::create_directories(dirPath);
	}

	currentFile.open(currentFilePath, std::ios::out | std::ios::trunc);
	if (!currentFile.is_open())
	{
		std::cout << "[" << get_timestamp() << "] [DataLogger] Failed to open CSV file: " << currentFilePath << std::endl;
		return false;
	}

	// Write headers
	for (size_t i = 0; i < headers.size(); ++i)
	{
		currentFile << headers[i];
		if (i < headers.size() - 1)
		{
			currentFile << delimiter;
		}
	}
	currentFile << "\n";
	currentFile.flush();

	std::cout << "[" << get_timestamp() << "] [DataLogger] Opened CSV file: " << currentFilePath << std::endl;
	return true;
}

void CsvWriter::close()
{
	std::lock_guard<std::mutex> lock(fileMutex);

	if (currentFile.is_open())
	{
		currentFile.close();
		std::cout << "[" << get_timestamp() << "] [DataLogger] Closed CSV file: " << currentFilePath << std::endl;
	}
}

bool CsvWriter::writeRow(const std::vector<std::string> &values)
{
	std::lock_guard<std::mutex> lock(fileMutex);

	if (!currentFile.is_open())
	{
		return false;
	}

	for (size_t i = 0; i < values.size(); ++i)
	{
		currentFile << values[i];
		if (i < values.size() - 1)
		{
			currentFile << delimiter;
		}
	}
	currentFile << "\n";
	currentFile.flush();

	return true;
}

bool CsvWriter::needsRotation() const
{
	std::lock_guard<std::mutex> lock(fileMutex);

	if (!currentFile.is_open())
	{
		return false;
	}

	return std::filesystem::file_size(currentFilePath) >= maxFileSizeBytes;
}

bool CsvWriter::rotate()
{
	std::vector<std::string> headers = columnHeaders;
	close();
	rotationCounter++;
	return open(headers);
}

std::string CsvWriter::getCurrentFilePath() const
{
	std::lock_guard<std::mutex> lock(fileMutex);
	return currentFilePath;
}

void CsvWriter::setDelimiter(const std::string &delim)
{
	delimiter = delim;
}

std::string CsvWriter::generateFileName() const
{
	std::filesystem::path base(basePath);
	std::string timestamp = generateTimestamp();
	std::string filename = timestamp;

	if (rotationCounter > 0)
	{
		filename += "_part" + std::to_string(rotationCounter + 1);
	}
	filename += ".csv";

	return (base / filename).string();
}

std::string CsvWriter::generateTimestamp() const
{
	auto now = std::chrono::system_clock::now();
	auto time_t_now = std::chrono::system_clock::to_time_t(now);

	std::tm localTime;
	localtime_s(&localTime, &time_t_now);

	std::ostringstream oss;
	oss << std::setfill('0')
	    << std::setw(2) << localTime.tm_mday << "-"
	    << std::setw(2) << localTime.tm_hour << "-"
	    << std::setw(2) << localTime.tm_min << "-"
	    << std::setw(2) << localTime.tm_sec;
	return oss.str();
}

// ArchiveManager Implementation

bool ArchiveManager::archiveFolder(const std::string &folderPath, const std::string &archiveFolderPath)
{
	// Archiving functionality - simplified version without external zip library
	// For now, just log that archiving would happen here
	// Full implementation would require a zip library like miniz or zlib
	if (!std::filesystem::exists(folderPath))
	{
		return true; // Nothing to archive
	}

	// Ensure archive directory exists
	if (!std::filesystem::exists(archiveFolderPath))
	{
		std::filesystem::create_directories(archiveFolderPath);
	}

	std::cout << "[" << get_timestamp() << "] [ArchiveManager] Archiving not fully implemented. Would archive: " 
	          << folderPath << " to " << archiveFolderPath << std::endl;
	return true;
}

std::string ArchiveManager::generateArchiveName()
{
	auto now = std::chrono::system_clock::now();
	auto time_t_now = std::chrono::system_clock::to_time_t(now);

	std::tm localTime;
	localtime_s(&localTime, &time_t_now);

	std::ostringstream oss;
	oss << std::setfill('0')
	    << (localTime.tm_year + 1900) << "-"
	    << std::setw(2) << (localTime.tm_mon + 1) << ".zip";
	return oss.str();
}

// DataLogger Implementation

DataLogger::DataLogger(std::shared_ptr<Settings> settings) :
  settings(settings)
{
}

DataLogger::~DataLogger()
{
	shutdown();
}

bool DataLogger::initialize()
{
	if (!settings->is_data_logging_enabled())
	{
		std::cout << "[" << get_timestamp() << "] [DataLogger] Data logging is disabled in settings." << std::endl;
		return true;
	}

	if (settings->is_archive_on_startup_enabled())
	{
		archiveOldFiles();
	}

	std::string dataPath = getDataFolderPath();
	std::uint32_t maxFileSizeMB = settings->get_data_logging_max_file_size_mb();

	csvWriter = std::make_unique<CsvWriter>(dataPath, maxFileSizeMB);
	csvWriter->setDelimiter(settings->get_csv_delimiter());

	// Calculate minimum log interval from frequency setting
	std::uint32_t frequencyHz = settings->get_data_logging_frequency_hz();
	minLogIntervalMs = 1000 / frequencyHz;

	lastLogTime = std::chrono::steady_clock::now() - std::chrono::milliseconds(minLogIntervalMs); // Allow first log immediately

	initialized = true;
	std::cout << "[" << get_timestamp() << "] [DataLogger] Initialized successfully. Logging at " << frequencyHz << " Hz (" << minLogIntervalMs << "ms interval)." << std::endl;
	return true;
}

void DataLogger::shutdown()
{
	if (csvWriter)
	{
		csvWriter->close();
		csvWriter.reset();
	}
	initialized = false;
}

void DataLogger::logDataFrame(const GpsData &gpsData, const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients)
{
	if (!initialized || !settings->is_data_logging_enabled())
	{
		return;
	}

	// Rate limiting - check if enough time has passed since last log
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastLogTime).count();
	if (elapsed < static_cast<std::int64_t>(minLogIntervalMs))
	{
		return; // Skip this frame, not enough time elapsed
	}
	lastLogTime = now;

	// Check if we need to create a new file (first run or rotation)
	if (!csvWriter || csvWriter->getCurrentFilePath().empty())
	{
		auto headers = buildHeaders(clients);
		if (!csvWriter->open(headers))
		{
			std::cout << "[" << get_timestamp() << "] [DataLogger] Failed to open CSV file for writing." << std::endl;
			return;
		}
	}

	// Check for rotation
	if (csvWriter->needsRotation())
	{
		if (!csvWriter->rotate())
		{
			std::cout << "[" << get_timestamp() << "] [DataLogger] Failed to rotate CSV file." << std::endl;
			return;
		}
	}

	// Build and write row
	auto row = buildRow(gpsData, clients);
	csvWriter->writeRow(row);
}

void DataLogger::updateDdiValue(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t value, const std::string &prettyName)
{
	if (!settings->is_data_logging_enabled())
	{
		return;
	}

	std::lock_guard<std::mutex> lock(ddiCacheMutex);

	std::string clientKey = std::to_string(client->get_NAME().get_full_name());
	std::string ddiKey = std::to_string(elementNumber) + "_" + std::to_string(ddi);

	DdiValue ddiValue;
	ddiValue.elementNumber = elementNumber;
	ddiValue.ddi = ddi;
	ddiValue.value = value;
	ddiValue.prettyName = prettyName;
	ddiValue.isValid = true;

	ddiCache[clientKey][ddiKey] = ddiValue;
}

bool DataLogger::isEnabled() const
{
	return settings->is_data_logging_enabled();
}

bool DataLogger::isBlockageMonitoringEnabled() const
{
	return settings->is_blockage_monitoring_enabled();
}

void DataLogger::archiveOldFiles()
{
	std::string dataPath = getDataFolderPath();
	std::string archivePath = getArchiveFolderPath();
	std::string logsPath = getLogsFolderPath();

	// Archive data folder
	ArchiveManager::archiveFolder(dataPath, archivePath);

	// Archive logs folder
	ArchiveManager::archiveFolder(logsPath, archivePath);
}

std::string DataLogger::getDataFolderPath() const
{
	return Settings::get_filename_path("data");
}

std::string DataLogger::getArchiveFolderPath() const
{
	return Settings::get_filename_path("archive");
}

std::string DataLogger::getLogsFolderPath() const
{
	return Settings::get_filename_path("logs");
}

std::vector<std::string> DataLogger::buildHeaders(const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients)
{
	std::vector<std::string> headers;

	// GPS/IMU data headers
	if (settings->is_log_gps_data_enabled())
	{
		headers.push_back("Date");
		headers.push_back("Latitude");
		headers.push_back("Longitude");
		headers.push_back("Height");
		headers.push_back("Speed");
		headers.push_back("DGPS_RTK_State");
		headers.push_back("DGPS_Age");
		headers.push_back("Roll");
		headers.push_back("HeadingDual");
		headers.push_back("HeadingTrue");
		headers.push_back("Satellites");
		headers.push_back("HDOP_x100");
		headers.push_back("IMU_Heading");
		headers.push_back("IMU_Roll");
		headers.push_back("IMU_Pitch");
		headers.push_back("IMU_Yaw");
	}

	// ISOBUS data headers
	if (settings->is_log_isobus_data_enabled())
	{
		for (const auto &client : clients)
		{
			const auto &state = client.second;
			std::string clientPrefix = "Client" + std::to_string(client.first->get_NAME().get_full_name()) + "_";

			// Section states
			for (std::uint8_t i = 0; i < state.get_number_of_sections(); ++i)
			{
				headers.push_back(clientPrefix + "SectionState_" + std::to_string(i + 1));
			}

			// Work state
			headers.push_back(clientPrefix + "WorkState");

			// DDI values from cache
			std::string clientKey = std::to_string(client.first->get_NAME().get_full_name());
			std::lock_guard<std::mutex> lock(ddiCacheMutex);
			auto it = ddiCache.find(clientKey);
			if (it != ddiCache.end())
			{
				for (const auto &ddiEntry : it->second)
				{
					headers.push_back(clientPrefix + getDdiColumnName(ddiEntry.second));
				}
			}
		}
	}

	return headers;
}

std::vector<std::string> DataLogger::buildRow(const GpsData &gpsData, const std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &clients)
{
	std::vector<std::string> row;
	auto now = std::chrono::system_clock::now();

	// GPS/IMU data
	if (settings->is_log_gps_data_enabled())
	{
		row.push_back(formatTimestamp(now));
		row.push_back(gpsData.isValid ? std::to_string(gpsData.latitude) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.longitude) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.height) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.speed) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.dgpsState) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.dgpsAge) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.roll) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.headingDual) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.headingTrue) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.satellites) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.hdopX100) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.imuHeading) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.imuRoll) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.imuPitch) : "");
		row.push_back(gpsData.isValid ? std::to_string(gpsData.imuYaw) : "");
	}

	// ISOBUS data
	if (settings->is_log_isobus_data_enabled())
	{
		for (const auto &client : clients)
		{
			const auto &state = client.second;

			// Section states
			for (std::uint8_t i = 0; i < state.get_number_of_sections(); ++i)
			{
				row.push_back(std::to_string(state.get_section_actual_state(i)));
			}

			// Work state
			row.push_back(state.get_actual_work_state() ? "1" : "0");

			// DDI values from cache
			std::string clientKey = std::to_string(client.first->get_NAME().get_full_name());
			std::lock_guard<std::mutex> lock(ddiCacheMutex);
			auto it = ddiCache.find(clientKey);
			if (it != ddiCache.end())
			{
				for (const auto &ddiEntry : it->second)
				{
					row.push_back(std::to_string(ddiEntry.second.value));
				}
			}
		}
	}

	return row;
}

std::string DataLogger::getDdiColumnName(const DdiValue &ddiValue) const
{
	// Format: Element{Number}_{PrettyName}__DDI{Number}
	std::string prettyName = ddiValue.prettyName;

	// Remove spaces and special characters from pretty name
	std::string cleanName;
	for (char c : prettyName)
	{
		if (std::isalnum(c))
		{
			cleanName += c;
		}
		else if (c == ' ')
		{
			cleanName += '_';
		}
	}

	return "Element" + std::to_string(ddiValue.elementNumber) + "_" + cleanName + "__DDI" + std::to_string(ddiValue.ddi);
}

std::string DataLogger::formatTimestamp(const std::chrono::system_clock::time_point &time) const
{
	auto time_t_now = std::chrono::system_clock::to_time_t(time);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()) % 1000;

	std::tm localTime;
	localtime_s(&localTime, &time_t_now);

	std::ostringstream oss;
	oss << std::setfill('0')
	    << std::setw(4) << (localTime.tm_year + 1900) << "-"
	    << std::setw(2) << (localTime.tm_mon + 1) << "-"
	    << std::setw(2) << localTime.tm_mday << " "
	    << std::setw(2) << localTime.tm_hour << ":"
	    << std::setw(2) << localTime.tm_min << ":"
	    << std::setw(2) << localTime.tm_sec << "."
	    << std::setw(3) << ms.count();
	return oss.str();
}
