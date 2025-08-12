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
 * @file    serial_port.hpp
 * @brief   Manages serial port communication for ERP42 platform
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_SERIAL_SERIAL_PORT_HPP_
#define ERP42_SERIAL_SERIAL_PORT_HPP_

#include <string>

namespace erp42
{
    /**
     * @brief Manages serial port communication.
     * @details Provides functionality to open, configure, read from, 
     * and write to a serial port using POSIX APIs.
     */
    class SerialPort
    {
    // "SerialPort" member functions
    public:

        /** 
         * @brief Default class constructor 
         * @param port_path Filesystem path to the serial device (e.g., "/dev/ttyUSB0").
         * @param baud_rate Communication speed in bits per second (e.g., 115200).
         */
        SerialPort(const std::string &port_path, const int &baud_rate);

        /** @brief Default class destructor */
        ~SerialPort() = default;

        /**
         * @brief Opens the serial port and initializes it.
         * @return 'true' if the port was opened and initialized successfully; 'false' otherwise.
         */
        void open_port();

        /**
         * @brief Closes the serial port.
         * @return 'true' if the port was closed successfully; 'false' otherwise.
         */
        void close_port();

        /**
         * @brief Initializes the serial port with 8N1 settings and the configured baud rate.
         * @return 'true' on successful configuration; 'false' otherwise.
         */
        void initialize_port();

        /**
         * @brief Receives a packet from the serial port.
         * @param rx_packet Pointer to the buffer where received data will be stored.
         * @param expected_packet_size Number of bytes expected to read.
         * @return 'true' if the received packet size matches 'expected_packet_size'; 'false' otherwise.
         */
        bool receive_packet(unsigned char *rx_packet, const unsigned int &expected_packet_size) const;

        /**
         * @brief Transmits a packet over the serial port.
         * @param tx_packet Pointer to the buffer containing the data to send.
         * @param expected_packet_size Number of bytes to transmit.
         * @return 'true' if the data was written successfully; 'false' otherwise.
         */
        bool transmit_packet(const unsigned char *tx_packet, const unsigned int &expected_packet_size) const;

    private:

        /** @brief Deleted copy constructor to prevent copying of SerialPort instances. */
        SerialPort(const SerialPort&) = delete;

        /** @brief Deleted copy assignment operator to prevent assignment of SerialPort instances. */
        SerialPort& operator=(const SerialPort&) = delete;

    // "SerialPort" member variables
    private:

        // Path to the serial device
        const std::string port_path_;

        // Serial communication baud rate
        const int baud_rate_;

        // File descriptor for the opened serial port device
        int file_descriptor_;

    }; // class SerialPort

} // namespace erp42

#endif // ERP42_SERIAL_SERIAL_PORT_HPP_