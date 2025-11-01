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
 * @file    log.cpp
 * @brief   Log for linux terminal and file system
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_util/log.hpp"
#include <unistd.h>
#include <iostream>
#include <cstdio>
#include <cstdarg>
#include <mutex>

namespace // anonymous namespace for internal linkage
{
    // Maximum size of formatted log message buffer
    #define MAX_BUFFER_SIZE 1024

    // Predefined log message headers for each log level
    static const char *log_header[4] = {"[ERP42 Debug]: "   ,
                                        "[ERP42 Info]: "    ,
                                        "[ERP42 Warning]: " ,
                                        "[ERP42 Error]: "   };

    // Terminal colors corresponding to each log level
    static const char *log_color [4] = {ANSI_COLOR_CYAN   ,
                                        ANSI_COLOR_GREEN  , 
                                        ANSI_COLOR_YELLOW ,
                                        ANSI_COLOR_RED    };

    /**
     * @brief Internal manager for log handlers and configuration
     * @details Maintains current log level, active handler type,
     * and provides thread-safe access via mutex.
     */
    struct LogManager
    {
        // Mutex for thread safety
        std::mutex sync_lock_;

        // Current global log level
        erp42_util::LogLevel log_level_;

        // Activated handler type
        erp42_util::LogType log_type_;

        // Pointer to active handler
        erp42_util::LogHandler *current_handler_;

        // Terminal log handler instance
        erp42_util::LogHandlerTerminal *terminal_log_handler_;

        //  File handler instance, if enabled
        erp42_util::LogHandlerFile *file_log_handler_;

        /**
         * @brief Constructs LogManager with default settings
         * @details Default level INFO, terminal handler active,
         *          file handler disabled.
         */
        LogManager()
        {
            // Default log level : Information
            log_level_ = erp42_util::LogLevel::INFO;

            // File log handler is not activated at default
            terminal_log_handler_ = new erp42_util::LogHandlerTerminal();
            file_log_handler_     = nullptr;

            // Default log handler is terminal log handler
            log_type_        = erp42_util::LogType::TERMINAL;
            current_handler_ = terminal_log_handler_;
        }

        /** @brief Destructs LogManager and releases handlers */
        ~LogManager()
        {
            // Deallocate all log output handlers
            delete terminal_log_handler_;
            delete file_log_handler_;

            // Set nullptr to each handlers
            current_handler_      = nullptr;
            terminal_log_handler_ = nullptr;
            file_log_handler_     = nullptr;
        }

    }; // struct LogManager

    /**
     * @brief Returns the singleton instance of LogManager
     * @return Pointer to the LogManager singleton
     */
    static LogManager *get_log_manager()
    {
        static LogManager log_manager;
        return &log_manager;
    }

    /** @brief Convenience macro to lock and retrieve LogManager */
    #define GET_LOG_MANAGER                                               \
            LogManager *log_manager = get_log_manager();                  \
            std::lock_guard<std::mutex> synclLock(log_manager->sync_lock_)

} // namespace annonymous

/**
 * @brief Prints a log message to the terminal
 * @param file_name Name of the source file
 * @param line_number Line number where the log call occurred
 * @param log_level Severity level
 * @param msg Formatted log message
 */
void erp42_util::LogHandlerTerminal::print_log(const char *file_name, int line_number, LogLevel log_level, const std::string &msg)
{
    // Linux system only
    // bool isTTY(isatty(fileno(stderr)) != 0);
    bool isTTY = true;

    if(erp42_util::LogLevel::WARNING <= log_level)
    {
        if(isTTY)
        {
            std::cerr << log_color[log_level] << log_header[log_level] 
                      << msg << " at line " << line_number << " in " << file_name
                      << ANSI_COLOR_RESET << std::endl;
            std::cerr.flush();
        }
    }
    else // log_level <= erp42_util::LogLevel::INFO
    {
        if(isTTY)
        {
            std::cout << log_color[log_level] << log_header[log_level]
                      << msg
                      << ANSI_COLOR_RESET << std::endl;
            std::cout.flush();
        }
    }
}

/**
 * @brief Constructor for file log handler
 * @param file_name Path to the log file
 */
erp42_util::LogHandlerFile::LogHandlerFile(const char *file_name)
{
    file_ = fopen(file_name, "a");
    if(file_ == nullptr)
    {
        std::cerr << "[LogHandlerFile]: Log file opening error" << std::endl;
    }
}

/** @brief Destructor that closes the log file */
erp42_util::LogHandlerFile::~LogHandlerFile()
{
    if(file_ != nullptr)
    {
        if(fclose(file_) != 0)
        {
            std::cerr << "[LogHandlerFile]: Log file closing error" << std::endl;
        }
    }
}

/**
 * @brief Prints a log message to the file
 * @param file_name Name of the source file
 * @param line_number Line number where the log call occurred
 * @param log_level Severity level
 * @param msg Formatted log message
 */
void erp42_util::LogHandlerFile::print_log(const char *file_name, int line_number, LogLevel log_level, const std::string &msg)
{
    if(file_ != nullptr)
    {
        if(erp42_util::LogLevel::WARNING <= log_level)
        {
            fprintf(file_, "%s%s", log_header[log_level], msg.c_str());
            fprintf(file_, " at line %d in %s\n", line_number, file_name);
        }
        else
        {
            fprintf(file_, "%s%s\n", log_header[log_level], msg.c_str());
        }
        fflush(file_);
    }
}

/**
 * @brief Sets the global log level
 * @param log_level Desired minimum severity level for logging
 */
void erp42_util::set_log_level(LogLevel log_level)
{
    GET_LOG_MANAGER;
    log_manager->log_level_ = log_level;
}

/**
 * @brief Retrieves the current global log level
 * @return Current minimum severity level for logging
 */
erp42_util::LogLevel erp42_util::get_log_level()
{
    GET_LOG_MANAGER;
    return log_manager->log_level_;
}

/**
 * @brief Enables file output handler for logging
 * @param file_name Path to the log file
 */
void erp42_util::activate_log_file(const std::string &file_name)
{
    GET_LOG_MANAGER;
    if(log_manager->file_log_handler_ == nullptr)
    {
        log_manager->file_log_handler_ = new LogHandlerFile(file_name.c_str());
    }
    else // (log_manager->m_LogHandlerFile != nullptr)
    {
        std::cerr << "[LogHandlerFile]: File log handler is already enabled" << std::endl;
    }
}

/** @brief Disables the file output handler and releases resources */
void erp42_util::deactivate_log_file()
{
    GET_LOG_MANAGER;
    if(log_manager->file_log_handler_ == nullptr)
    {
        std::cerr << "[LogHandlerFile]: File log handler is already disabled" << std::endl;
    }
    else // (log_manager->m_LogHandlerFile != nullptr)
    {
        delete log_manager->file_log_handler_;
        log_manager->file_log_handler_ = nullptr;
    }
}

/**
 * @brief Sets the active log handler type
 * @param log_type LogType::TERMINAL or LogType::FILE
 */
void erp42_util::set_log_type(LogType log_type)
{
    GET_LOG_MANAGER;
    if(log_type != log_manager->log_type_)
    {
        if(log_type == LogType::FILE)
        {
            if(log_manager->file_log_handler_ == nullptr)
            {
                std::cerr << "[LogHandlerFile]: File log handler is not allocated" << std::endl;
            }
            else
            {
                log_manager->log_type_ = LogType::FILE;
                log_manager->current_handler_ = log_manager->file_log_handler_;
            }
        }
        else // log_type == erp42_util::TERMINAL_HANDLER
        {
            log_manager->log_type_ = LogType::TERMINAL;
            log_manager->current_handler_ = log_manager->terminal_log_handler_;
        }
    }
}

/**
 * @brief Retrieves the active log handler type
 * @return Currently configured LogType
 */
erp42_util::LogType erp42_util::get_log_type()
{
    GET_LOG_MANAGER;
    return log_manager->log_type_;
}

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
void erp42_util::print_log(const char *file_name, int line_number, LogLevel log_level, const char *msg, ...)
{
    GET_LOG_MANAGER;
    if(log_manager->current_handler_ != nullptr && log_manager->log_level_ <= log_level)
    {
        va_list args;
        va_start(args, msg);

            char buffer[MAX_BUFFER_SIZE];
            vsnprintf(buffer, sizeof(buffer), msg, args);

        va_end(args);

        buffer[MAX_BUFFER_SIZE - 1] = '\0';
        log_manager->current_handler_->print_log(file_name, line_number, log_level, buffer);
    }
}