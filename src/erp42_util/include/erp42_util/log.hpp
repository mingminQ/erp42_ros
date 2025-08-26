/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file    log.hpp
 * @brief   Log for linux terminal and file system
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_UTIL_LOG_HPP_
#define ERP42_UTIL_LOG_HPP_

#include <string>

// Terminal log handler macro
#define ERP42_DEBUG(msg, ...) erp42::log::print_log(__FILE__, __LINE__, erp42::log::LogLevel::DEBUG   , msg, ##__VA_ARGS__)
#define ERP42_INFO(msg, ...)  erp42::log::print_log(__FILE__, __LINE__, erp42::log::LogLevel::INFO    , msg, ##__VA_ARGS__)
#define ERP42_WARN(msg, ...)  erp42::log::print_log(__FILE__, __LINE__, erp42::log::LogLevel::WARNING , msg, ##__VA_ARGS__)
#define ERP42_ERROR(msg, ...) erp42::log::print_log(__FILE__, __LINE__, erp42::log::LogLevel::ERROR   , msg, ##__VA_ARGS__)

namespace erp42
{
    namespace log
    {
        // Types of log output
        enum class LogType
        {
            TERMINAL = 0, // Log output to terminal
            FILE     = 1  // Log output to file

        }; // enum class HandlerType

        // Log level enumeration
        enum LogLevel
        {
            DEBUG   = 0, // Debug level
            INFO    = 1, // Information level
            WARNING = 2, // Warning level
            ERROR   = 3  // Error level

        }; // enum LogLevel

        /**
         * @brief Abstract base class for log handlers
         * @details Defines the interface for outputting log messages
         * to various destinations (e.g., terminal, file).
         */
        class LogHandler
        {
        // "LogHandler" member functions
        public:

            /** @brief Default class constructor */
            LogHandler() = default;

            /** @brief Default class destructor */
            virtual ~LogHandler() = default;

            /**
             * @brief Outputs a log message
             * @param file_name Name of the source file where the log was called
             * @param line_number Line number in the source file
             * @param log_level Severity level of the log message
             * @param msg Content of the log message
             */
            virtual void print_log(const char *file_name, int line_number, LogLevel log_level, const std::string &msg) = 0;

        }; // class LogHandler

        /**
         * @brief Terminal log handler
         * @details Outputs log messages to the Linux terminal with formatting.
         */
        class LogHandlerTerminal : public LogHandler
        {
        // "LogHandlerTerminal" member functions
        public:

            /** @brief Default class constructor */
            LogHandlerTerminal() = default;

            /**
             * @brief Prints a log message to the terminal
             * @param file_name Name of the source file
             * @param line_number Line number where the log call occurred
             * @param log_level Severity level
             * @param msg Formatted log message
             */
            void print_log(const char *file_name, int line_number, LogLevel log_level, const std::string &msg) override;

        }; // class LogHandlerTerminal

        /**
         * @brief File log handler
         * @details Appends log messages to a specified file.
         */
        class LogHandlerFile : public LogHandler
        {
        // "LogHandlerFile" member functions
        public:

            /**
             * @brief Constructor for file log handler
             * @param file_name Path to the log file
             */
            LogHandlerFile(const char *file_name);

            /** @brief Destructor that closes the log file */
            ~LogHandlerFile() override;

            /**
             * @brief Prints a log message to the file
             * @param file_name Name of the source file
             * @param line_number Line number where the log call occurred
             * @param log_level Severity level
             * @param msg Formatted log message
             */
            void print_log(const char *file_name, int line_number, LogLevel log_level, const std::string &msg) override;
        
        // "LogHandlerFile" member variables
        private:

            // File pointer
            FILE *file_;
        
        }; // class LogHandlerFile

        /**
         * @brief Sets the global log level
         * @param log_level Desired minimum severity level for logging
         */
        void set_log_level(LogLevel log_level);

        /**
         * @brief Retrieves the current global log level
         * @return Current minimum severity level for logging
         */
        LogLevel get_log_level();

        /**
         * @brief Enables file output handler for logging
         * @param file_name Path to the log file
         */
        void activate_log_file(const std::string &file_name);

        /** @brief Disables the file output handler and releases resources */
        void deactivate_log_file();

        /**
         * @brief Sets the active log handler type
         * @param handler_type HandlerType::TERMINAL or HandlerType::FILE
         */
        void set_log_type(LogType handler_type);

        /**
         * @brief Retrieves the active log handler type
         * @return Currently configured HandlerType
         */
        LogType get_log_type();

        /**
         * @brief Core logging function
         * @details Routes the log message to the configured handler
         * and applies the global log level filter.
         * @param file_name Source file name
         * @param line_number Source file line number
         * @param log_level Severity level of the message
         * @param msg Format string for the log message
         * @param ... Additional arguments for formatting
         */
        void print_log(const char *file_name, int line_number, LogLevel log_level, const char *msg, ...);

    } // namespace log

} // namespace erp42

#endif // ERP42_UTIL_LOG_HPP_