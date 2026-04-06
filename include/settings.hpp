/**
 * @author Daan Steenbergen
 * @brief An interface to store/load AOG-TC settings to/from a file
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include <array>
#include <string>

/// @brief A class to store/load AOG-TC settings to/from a file
class Settings
{
public:
	/**
     * @brief (re)load the settings from specified file
     * @return True if the settings were loaded successfully, false otherwise
     */
	bool load();

	/**
     * @brief Save the settings to specified file
     * @return True if the settings were saved successfully, false otherwise
     */
	bool save() const;

	/**
	 * @brief Get the configured subnet
	 * @return The configured subnet
	 */
	const std::array<std::uint8_t, 3> &get_subnet() const;

	/**
	 * @brief Get the configured subnet as a string
	 * @return The configured subnet as a string
	 */
	std::string get_subnet_string() const;

	/**
	 * @brief Set the configured subnet
	 * @param subnet The subnet to set
	 * @param save Whether or not to save the settings to file
	 * @return True if the subnet was set successfully, false otherwise
	 */
	bool set_subnet(std::array<std::uint8_t, 3> subnet, bool save = true);

	/**
	 * @brief Check if Tractor ECU is enabled
	 * @return True if TECU is enabled, false otherwise
	 */
	bool is_tecu_enabled() const;

	/**
	 * @brief Set the Tractor ECU enabled state
	 * @param enabled Whether to enable the Tractor ECU
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_tecu_enabled(bool enabled, bool save = true);

	/**
	 * @brief Check if data logging is enabled
	 * @return True if data logging is enabled, false otherwise
	 */
	bool is_data_logging_enabled() const;

	/**
	 * @brief Set the data logging enabled state
	 * @param enabled Whether to enable data logging
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_data_logging_enabled(bool enabled, bool save = true);

	/**
	 * @brief Get the maximum file size for data logging in MB
	 * @return The maximum file size in MB
	 */
	std::uint32_t get_data_logging_max_file_size_mb() const;

	/**
	 * @brief Set the maximum file size for data logging
	 * @param sizeMB The maximum file size in MB
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_data_logging_max_file_size_mb(std::uint32_t sizeMB, bool save = true);

	/**
	 * @brief Check if archive on startup is enabled
	 * @return True if archive on startup is enabled, false otherwise
	 */
	bool is_archive_on_startup_enabled() const;

	/**
	 * @brief Set the archive on startup enabled state
	 * @param enabled Whether to enable archive on startup
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_archive_on_startup_enabled(bool enabled, bool save = true);

	/**
	 * @brief Check if blockage monitoring is enabled
	 * @return True if blockage monitoring is enabled, false otherwise
	 */
	bool is_blockage_monitoring_enabled() const;

	/**
	 * @brief Set the blockage monitoring enabled state
	 * @param enabled Whether to enable blockage monitoring
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_blockage_monitoring_enabled(bool enabled, bool save = true);

	/**
	 * @brief Check if GPS data logging is enabled
	 * @return True if GPS data logging is enabled, false otherwise
	 */
	bool is_log_gps_data_enabled() const;

	/**
	 * @brief Set the GPS data logging enabled state
	 * @param enabled Whether to enable GPS data logging
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_log_gps_data_enabled(bool enabled, bool save = true);

	/**
	 * @brief Check if ISOBUS data logging is enabled
	 * @return True if ISOBUS data logging is enabled, false otherwise
	 */
	bool is_log_isobus_data_enabled() const;

	/**
	 * @brief Set the ISOBUS data logging enabled state
	 * @param enabled Whether to enable ISOBUS data logging
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_log_isobus_data_enabled(bool enabled, bool save = true);

	/**
	 * @brief Get the CSV delimiter
	 * @return The CSV delimiter string
	 */
	std::string get_csv_delimiter() const;

	/**
	 * @brief Set the CSV delimiter
	 * @param delimiter The delimiter to use
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_csv_delimiter(const std::string &delimiter, bool save = true);

	/**
	 * @brief Get the date format string
	 * @return The date format string (strftime format)
	 */
	std::string get_date_format() const;

	/**
	 * @brief Set the date format string
	 * @param format The date format (strftime format)
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_date_format(const std::string &format, bool save = true);

	/**
	 * @brief Get the time format string
	 * @return The time format string (strftime format)
	 */
	std::string get_time_format() const;

	/**
	 * @brief Set the time format string
	 * @param format The time format (strftime format)
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_time_format(const std::string &format, bool save = true);

	/**
	 * @brief Get the decimal separator
	 * @return The decimal separator string (e.g., "." or ",")
	 */
	std::string get_decimal_separator() const;

	/**
	 * @brief Set the decimal separator
	 * @param separator The decimal separator (e.g., "." or ",")
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_decimal_separator(const std::string &separator, bool save = true);

	/**
	 * @brief Get the locale setting
	 * @return The locale string (e.g., "en-US", "de-DE")
	 */
	std::string get_locale() const;

	/**
	 * @brief Set the locale setting
	 * @param locale The locale string (e.g., "en-US", "de-DE")
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_locale(const std::string &locale, bool save = true);

	/**
	 * @brief Get the ISOBUS language code from locale
	 * @return The ISOBUS language code (0-255) for DDI 95
	 */
	std::uint8_t get_isobus_language_code() const;

	/**
	 * @brief Get the data logging frequency in Hz
	 * @return The logging frequency (entries per second)
	 */
	std::uint32_t get_data_logging_frequency_hz() const;

	/**
	 * @brief Set the data logging frequency in Hz
	 * @param frequencyHz The logging frequency (entries per second, 1-100)
	 * @param save Whether or not to save the settings to file
	 * @return True if the setting was set successfully, false otherwise
	 */
	bool set_data_logging_frequency_hz(std::uint32_t frequencyHz, bool save = true);

	/**
	 * @brief Get the absolute path to the settings file
	 * @param filename The filename to get the path for
	 * @return The absolute path to the settings file
	 */
	static std::string get_filename_path(std::string);

	/**
	 * @brief Check if settings file exists
	 * @return True if settings file exists, false otherwise
	 */
	bool exists() const;

private:
	constexpr static std::array<std::uint8_t, 3> DEFAULT_SUBNET = { 192, 168, 5 };
	constexpr static bool DEFAULT_TECU_ENABLED = true;
	constexpr static bool DEFAULT_DATA_LOGGING_ENABLED = true;
	constexpr static std::uint32_t DEFAULT_DATA_LOGGING_MAX_FILE_SIZE_MB = 100;
	constexpr static bool DEFAULT_ARCHIVE_ON_STARTUP = true;
	constexpr static bool DEFAULT_BLOCKAGE_MONITORING_ENABLED = true;
	constexpr static bool DEFAULT_LOG_GPS_DATA_ENABLED = true;
	constexpr static bool DEFAULT_LOG_ISOBUS_DATA_ENABLED = true;
	constexpr static const char *DEFAULT_CSV_DELIMITER = ";";
	constexpr static const char *DEFAULT_DATE_FORMAT = "%Y-%m-%d";
	constexpr static const char *DEFAULT_TIME_FORMAT = "%H:%M:%S";
	constexpr static const char *DEFAULT_DECIMAL_SEPARATOR = ".";
	constexpr static const char *DEFAULT_LOCALE = "en-US";
	constexpr static std::uint32_t DEFAULT_DATA_LOGGING_FREQUENCY_HZ = 10;

	std::array<std::uint8_t, 3> configuredSubnet = DEFAULT_SUBNET;
	bool tecuEnabled = DEFAULT_TECU_ENABLED;
	bool dataLoggingEnabled = DEFAULT_DATA_LOGGING_ENABLED;
	std::uint32_t dataLoggingMaxFileSizeMB = DEFAULT_DATA_LOGGING_MAX_FILE_SIZE_MB;
	bool archiveOnStartup = DEFAULT_ARCHIVE_ON_STARTUP;
	bool blockageMonitoringEnabled = DEFAULT_BLOCKAGE_MONITORING_ENABLED;
	bool logGpsDataEnabled = DEFAULT_LOG_GPS_DATA_ENABLED;
	bool logIsobusDataEnabled = DEFAULT_LOG_ISOBUS_DATA_ENABLED;
	std::string csvDelimiter = DEFAULT_CSV_DELIMITER;
	std::string dateFormat = DEFAULT_DATE_FORMAT;
	std::string timeFormat = DEFAULT_TIME_FORMAT;
	std::string decimalSeparator = DEFAULT_DECIMAL_SEPARATOR;
	std::string locale = DEFAULT_LOCALE;
	std::uint32_t dataLoggingFrequencyHz = DEFAULT_DATA_LOGGING_FREQUENCY_HZ;
};
