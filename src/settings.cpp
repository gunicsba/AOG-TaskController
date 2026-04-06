/**
 * @author Daan Steenbergen
 * @brief An interface to store/load AOG-TC settings to/from a file
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "settings.hpp"

#include <ShlObj_core.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

bool Settings::exists() const
{
	std::ifstream file(get_filename_path("settings.json"));
	return file.is_open();
}

bool Settings::load()
{
	std::ifstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		// Create default settings file if it doesn't exist
		std::cout << "Settings file not found, creating default settings..." << std::endl;
		return save();
	}

	json data;
	file >> data;

	if (data.contains("subnet"))
	{
		try
		{
			auto subnetData = data["subnet"].get<std::array<int, 3>>(); // Directly get the array
			std::copy(subnetData.begin(), subnetData.end(), configuredSubnet.begin());
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'subnet': " << e.what() << std::endl;
			configuredSubnet = DEFAULT_SUBNET; // Fallback to default
		}
	}
	else
	{
		configuredSubnet = DEFAULT_SUBNET; // Key not found, use default
	}

	if (data.contains("tecuEnabled"))
	{
		try
		{
			tecuEnabled = data["tecuEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'tecuEnabled': " << e.what() << std::endl;
			tecuEnabled = DEFAULT_TECU_ENABLED; // Fallback to default
		}
	}
	else
	{
		tecuEnabled = DEFAULT_TECU_ENABLED; // Key not found, use default
	}

	if (data.contains("dataLoggingEnabled"))
	{
		try
		{
			dataLoggingEnabled = data["dataLoggingEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'dataLoggingEnabled': " << e.what() << std::endl;
			dataLoggingEnabled = DEFAULT_DATA_LOGGING_ENABLED;
		}
	}
	else
	{
		dataLoggingEnabled = DEFAULT_DATA_LOGGING_ENABLED;
	}

	if (data.contains("dataLoggingMaxFileSizeMB"))
	{
		try
		{
			dataLoggingMaxFileSizeMB = data["dataLoggingMaxFileSizeMB"].get<std::uint32_t>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'dataLoggingMaxFileSizeMB': " << e.what() << std::endl;
			dataLoggingMaxFileSizeMB = DEFAULT_DATA_LOGGING_MAX_FILE_SIZE_MB;
		}
	}
	else
	{
		dataLoggingMaxFileSizeMB = DEFAULT_DATA_LOGGING_MAX_FILE_SIZE_MB;
	}

	if (data.contains("archiveOnStartup"))
	{
		try
		{
			archiveOnStartup = data["archiveOnStartup"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'archiveOnStartup': " << e.what() << std::endl;
			archiveOnStartup = DEFAULT_ARCHIVE_ON_STARTUP;
		}
	}
	else
	{
		archiveOnStartup = DEFAULT_ARCHIVE_ON_STARTUP;
	}

	if (data.contains("blockageMonitoringEnabled"))
	{
		try
		{
			blockageMonitoringEnabled = data["blockageMonitoringEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'blockageMonitoringEnabled': " << e.what() << std::endl;
			blockageMonitoringEnabled = DEFAULT_BLOCKAGE_MONITORING_ENABLED;
		}
	}
	else
	{
		blockageMonitoringEnabled = DEFAULT_BLOCKAGE_MONITORING_ENABLED;
	}

	if (data.contains("logGpsDataEnabled"))
	{
		try
		{
			logGpsDataEnabled = data["logGpsDataEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'logGpsDataEnabled': " << e.what() << std::endl;
			logGpsDataEnabled = DEFAULT_LOG_GPS_DATA_ENABLED;
		}
	}
	else
	{
		logGpsDataEnabled = DEFAULT_LOG_GPS_DATA_ENABLED;
	}

	if (data.contains("logIsobusDataEnabled"))
	{
		try
		{
			logIsobusDataEnabled = data["logIsobusDataEnabled"].get<bool>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'logIsobusDataEnabled': " << e.what() << std::endl;
			logIsobusDataEnabled = DEFAULT_LOG_ISOBUS_DATA_ENABLED;
		}
	}
	else
	{
		logIsobusDataEnabled = DEFAULT_LOG_ISOBUS_DATA_ENABLED;
	}

	if (data.contains("csvDelimiter"))
	{
		try
		{
			csvDelimiter = data["csvDelimiter"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'csvDelimiter': " << e.what() << std::endl;
			csvDelimiter = DEFAULT_CSV_DELIMITER;
		}
	}
	else
	{
		csvDelimiter = DEFAULT_CSV_DELIMITER;
	}

	if (data.contains("dateFormat"))
	{
		try
		{
			dateFormat = data["dateFormat"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'dateFormat': " << e.what() << std::endl;
			dateFormat = DEFAULT_DATE_FORMAT;
		}
	}
	else
	{
		dateFormat = DEFAULT_DATE_FORMAT;
	}

	if (data.contains("timeFormat"))
	{
		try
		{
			timeFormat = data["timeFormat"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'timeFormat': " << e.what() << std::endl;
			timeFormat = DEFAULT_TIME_FORMAT;
		}
	}
	else
	{
		timeFormat = DEFAULT_TIME_FORMAT;
	}

	if (data.contains("decimalSeparator"))
	{
		try
		{
			decimalSeparator = data["decimalSeparator"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'decimalSeparator': " << e.what() << std::endl;
			decimalSeparator = DEFAULT_DECIMAL_SEPARATOR;
		}
	}
	else
	{
		decimalSeparator = DEFAULT_DECIMAL_SEPARATOR;
	}

	if (data.contains("locale"))
	{
		try
		{
			locale = data["locale"].get<std::string>();
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'locale': " << e.what() << std::endl;
			locale = DEFAULT_LOCALE;
		}
	}
	else
	{
		locale = DEFAULT_LOCALE;
	}

	if (data.contains("dataLoggingFrequencyHz"))
	{
		try
		{
			dataLoggingFrequencyHz = data["dataLoggingFrequencyHz"].get<std::uint32_t>();
			// Clamp to valid range 1-100 Hz
			if (dataLoggingFrequencyHz < 1)
				dataLoggingFrequencyHz = 1;
			if (dataLoggingFrequencyHz > 100)
				dataLoggingFrequencyHz = 100;
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'dataLoggingFrequencyHz': " << e.what() << std::endl;
			dataLoggingFrequencyHz = DEFAULT_DATA_LOGGING_FREQUENCY_HZ;
		}
	}
	else
	{
		dataLoggingFrequencyHz = DEFAULT_DATA_LOGGING_FREQUENCY_HZ;
	}

	return true;
}

bool Settings::save() const
{
	json data;
	data["subnet"] = configuredSubnet;
	data["tecuEnabled"] = tecuEnabled;
	data["dataLoggingEnabled"] = dataLoggingEnabled;
	data["dataLoggingMaxFileSizeMB"] = dataLoggingMaxFileSizeMB;
	data["archiveOnStartup"] = archiveOnStartup;
	data["blockageMonitoringEnabled"] = blockageMonitoringEnabled;
	data["logGpsDataEnabled"] = logGpsDataEnabled;
	data["logIsobusDataEnabled"] = logIsobusDataEnabled;
	data["csvDelimiter"] = csvDelimiter;
	data["dateFormat"] = dateFormat;
	data["timeFormat"] = timeFormat;
	data["decimalSeparator"] = decimalSeparator;
	data["locale"] = locale;
	data["dataLoggingFrequencyHz"] = dataLoggingFrequencyHz;

	std::ofstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		return false;
	}

	file << data.dump(4); // Pretty print
	return true;
}

const std::array<std::uint8_t, 3> &Settings::get_subnet() const
{
	return configuredSubnet;
}

std::string Settings::get_subnet_string() const
{
	return std::to_string(configuredSubnet[0]) + '.' + std::to_string(configuredSubnet[1]) + '.' + std::to_string(configuredSubnet[2]) + ".0";
}

bool Settings::set_subnet(std::array<std::uint8_t, 3> subnet, bool save)
{
	configuredSubnet = subnet;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_tecu_enabled() const
{
	return tecuEnabled;
}

bool Settings::set_tecu_enabled(bool enabled, bool save)
{
	tecuEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_data_logging_enabled() const
{
	return dataLoggingEnabled;
}

bool Settings::set_data_logging_enabled(bool enabled, bool save)
{
	dataLoggingEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::uint32_t Settings::get_data_logging_max_file_size_mb() const
{
	return dataLoggingMaxFileSizeMB;
}

bool Settings::set_data_logging_max_file_size_mb(std::uint32_t sizeMB, bool save)
{
	dataLoggingMaxFileSizeMB = sizeMB;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_archive_on_startup_enabled() const
{
	return archiveOnStartup;
}

bool Settings::set_archive_on_startup_enabled(bool enabled, bool save)
{
	archiveOnStartup = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_blockage_monitoring_enabled() const
{
	return blockageMonitoringEnabled;
}

bool Settings::set_blockage_monitoring_enabled(bool enabled, bool save)
{
	blockageMonitoringEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_log_gps_data_enabled() const
{
	return logGpsDataEnabled;
}

bool Settings::set_log_gps_data_enabled(bool enabled, bool save)
{
	logGpsDataEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

bool Settings::is_log_isobus_data_enabled() const
{
	return logIsobusDataEnabled;
}

bool Settings::set_log_isobus_data_enabled(bool enabled, bool save)
{
	logIsobusDataEnabled = enabled;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_csv_delimiter() const
{
	return csvDelimiter;
}

bool Settings::set_csv_delimiter(const std::string &delimiter, bool save)
{
	csvDelimiter = delimiter;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_date_format() const
{
	return dateFormat;
}

bool Settings::set_date_format(const std::string &format, bool save)
{
	dateFormat = format;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_time_format() const
{
	return timeFormat;
}

bool Settings::set_time_format(const std::string &format, bool save)
{
	timeFormat = format;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_decimal_separator() const
{
	return decimalSeparator;
}

bool Settings::set_decimal_separator(const std::string &separator, bool save)
{
	decimalSeparator = separator;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_locale() const
{
	return locale;
}

bool Settings::set_locale(const std::string &loc, bool save)
{
	locale = loc;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::uint32_t Settings::get_data_logging_frequency_hz() const
{
	return dataLoggingFrequencyHz;
}

bool Settings::set_data_logging_frequency_hz(std::uint32_t frequencyHz, bool save)
{
	// Clamp to valid range 1-100 Hz
	if (frequencyHz < 1)
		frequencyHz = 1;
	if (frequencyHz > 100)
		frequencyHz = 100;
	dataLoggingFrequencyHz = frequencyHz;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::uint8_t Settings::get_isobus_language_code() const
{
	// Map locale to ISOBUS language code (ISO 639-1 / ISO 11783-7)
	// Extract language part from locale (e.g., "en-US" -> "en")
	std::string lang = locale;
	size_t dashPos = lang.find('-');
	if (dashPos != std::string::npos)
	{
		lang = lang.substr(0, dashPos);
	}

	// Convert to lowercase for comparison
	std::transform(lang.begin(), lang.end(), lang.begin(), ::tolower);

	// ISOBUS language codes based on ISO 639-1
	// Reference: ISO 11783-7 DDI 95 Language
	if (lang == "en")
		return 0; // English
	if (lang == "de")
		return 1; // German
	if (lang == "fr")
		return 2; // French
	if (lang == "da")
		return 3; // Danish
	if (lang == "sv")
		return 4; // Swedish
	if (lang == "it")
		return 5; // Italian
	if (lang == "es")
		return 6; // Spanish
	if (lang == "nl")
		return 7; // Dutch
	if (lang == "fi")
		return 8; // Finnish
	if (lang == "no")
		return 9; // Norwegian
	if (lang == "pt")
		return 10; // Portuguese
	if (lang == "pl")
		return 11; // Polish
	if (lang == "cs")
		return 12; // Czech
	if (lang == "sk")
		return 13; // Slovak
	if (lang == "hu")
		return 14; // Hungarian
	if (lang == "sl")
		return 15; // Slovenian
	if (lang == "hr")
		return 16; // Croatian
	if (lang == "ro")
		return 17; // Romanian
	if (lang == "bg")
		return 18; // Bulgarian
	if (lang == "et")
		return 19; // Estonian
	if (lang == "lv")
		return 20; // Latvian
	if (lang == "lt")
		return 21; // Lithuanian
	if (lang == "ru")
		return 22; // Russian
	if (lang == "tr")
		return 23; // Turkish
	if (lang == "el")
		return 24; // Greek
	if (lang == "uk")
		return 25; // Ukrainian
	if (lang == "sr")
		return 26; // Serbian
	if (lang == "ja")
		return 27; // Japanese
	if (lang == "zh")
		return 28; // Chinese
	if (lang == "ko")
		return 29; // Korean
	if (lang == "ar")
		return 30; // Arabic
	if (lang == "he")
		return 31; // Hebrew
	if (lang == "th")
		return 32; // Thai
	if (lang == "hi")
		return 33; // Hindi
	if (lang == "id")
		return 34; // Indonesian
	if (lang == "ms")
		return 35; // Malay
	if (lang == "vi")
		return 36; // Vietnamese

	// Default to English (0) for unknown locales
	return 0;
}

std::string Settings::get_filename_path(std::string fileName)
{
	char path[MAX_PATH];
	if (SHGetFolderPath(NULL, CSIDL_APPDATA, NULL, 0, path) != S_OK)
	{
		throw std::runtime_error("Failed to get AppData path");
	}

	std::string baseDir = std::string(path) + "\\" + PROJECT_NAME;
	std::string fullPath = baseDir + "\\" + fileName;

	// Find the last directory separator (before the actual file name)
	size_t lastSlash = fullPath.find_last_of("\\/");
	if (lastSlash != std::string::npos)
	{
		std::string directoryPath = fullPath.substr(0, lastSlash); // Extract the directory part

		// Create each directory level iteratively
		std::istringstream dirStream(directoryPath);
		std::string segment;
		std::string currentPath;

		while (std::getline(dirStream, segment, '\\')) // Split by `\`
		{
			if (!currentPath.empty())
				currentPath += "\\"; // Append separator only after first segment

			currentPath += segment;

			if (CreateDirectory(currentPath.c_str(), NULL) == 0)
			{
				DWORD error = GetLastError();
				if (error != ERROR_ALREADY_EXISTS)
				{
					throw std::runtime_error("Failed to create directory: " + currentPath);
				}
			}
		}
	}

	return fullPath;
}
