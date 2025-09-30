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
 * @file    serial_port.cpp
 * @brief   Manages serial port communication for ERP42 platform
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "erp42_serial/serial_port.hpp"
#include "erp42_util/exception.hpp"
#include "erp42_util/log.hpp"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

/** 
 * @brief Default class constructor 
 * @param port_path Filesystem path to the serial device (e.g., "/dev/ttyUSB0").
 * @param baud_rate Communication speed in bits per second (e.g., 115200).
 */
erp42::SerialPort::SerialPort(const std::string &port_path, const int &baud_rate)
  : port_path_(port_path),
    baud_rate_(baud_rate),
    file_descriptor_(-1)
{
}

/**
 * @brief Opens and initializes the serial port.
 * @details Attempts to open the port specified by @c port_path_. 
 * Throws an exception if the path is empty or opening fails.
 * On success, calls @c initialize_port() to configure the port.
 */
void erp42::SerialPort::open_port()
{
    if(port_path_.empty())
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::open_port() Port path is empty.");
    }

    file_descriptor_ = open(port_path_.c_str(), O_RDWR | O_NOCTTY);
    if(file_descriptor_ == -1)
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::open_port() File descriptor opening error.");
    }

    this->initialize_port();
    ERP42_INFO("SerialPort::open_port() Serial port %s has been opened successfully.",
        port_path_.c_str());
}

/**
 * @brief Closes the serial port.
 * @details Closes the file descriptor associated with the serial port.
 * Throws an exception if the descriptor is invalid or closing fails.
 */
void erp42::SerialPort::close_port()
{
    if (file_descriptor_ < 0) 
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::close_port() Invalid file descriptor.");
    }

    if (close(file_descriptor_) != 0) 
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::close_port() Failed to close file descriptor.");
    }

    file_descriptor_ = -1;
    ERP42_INFO("SerialPort::close_port() Serial port %s has been closed successfully.",
        port_path_.c_str());
}

/**
 * @brief Receives a packet from the serial port.
 * @param rx_packet Pointer to the buffer where received data will be stored.
 * @param expected_packet_size Number of bytes expected to read.
 * @return 'true' if the received packet size matches 'expected_packet_size'; 'false' otherwise.
 */
bool erp42::SerialPort::receive_packet(unsigned char *rx_packet, const unsigned int &expected_packet_size) const
{
    if(file_descriptor_ < 0)
    {
        ERP42_ERROR("SerialPort::receive_packet() Port is not opened.");
        return false;
    }

    ssize_t received_packet_size = read(file_descriptor_, rx_packet, expected_packet_size);
    if(received_packet_size != expected_packet_size)
    {
        ERP42_ERROR("SerialPort::receive_packet() Expected size is %d, but received size is %zd.",
            expected_packet_size, received_packet_size);
        return false;
    }

    return true;
}

/**
 * @brief Transmits a packet over the serial port.
 * @param tx_packet Pointer to the buffer containing the data to send.
 * @param expected_packet_size Number of bytes to transmit.
 * @return 'true' if the data was written successfully; 'false' otherwise.
 */
bool erp42::SerialPort::transmit_packet(
    const unsigned char *tx_packet, const unsigned int &expected_packet_size
) const
{
    if(file_descriptor_ < 0)
    {
        ERP42_ERROR("SerialPort::transmit_packet() Port is not opened.");
        return false;
    }
    
    ssize_t transmitted_packet_size = write(file_descriptor_, tx_packet, expected_packet_size);
    if(transmitted_packet_size != expected_packet_size)
    {
        ERP42_ERROR("SerialPort::transmit_packet() Expected size is %d, but transmitted size is %zd.",
            expected_packet_size, transmitted_packet_size);
        return false;
    }

    return true;
}


/**
 * @brief Initializes the serial port with 8N1 format and the configured baud rate.
 * @details Configures the port using termios with raw mode, 8 data bits, no parity,
 * and 1 stop bit. Supports baud rates 9600 and 115200. 
 * Throws an exception if configuration fails or the baud rate is invalid.
 */
void erp42::SerialPort::initialize_port()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(file_descriptor_, &tty) != 0)
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::initialize_port() tcgetattr failed.");
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON   | IXOFF  | IXANY        );
    tty.c_iflag &= ~(BRKINT | ICRNL  | IXON         );
    tty.c_lflag &= ~(ECHO   | ICANON | IEXTEN | ISIG);

    tty.c_oflag &= ~OPOST;

    switch (baud_rate_)
    {
    case 9600:
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        break;

    case 115200:
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        break;

    default:
        file_descriptor_ = -1;
        throw Exception("SerialPort::initialize_port() Invalid baud_rate, use 9600 or 115200.");
    }

    tty.c_cc[VMIN]  = 18;
    tty.c_cc[VTIME] = 1;

    if((tcsetattr(file_descriptor_, TCSANOW, &tty)) != 0)
    {
        file_descriptor_ = -1;
        throw Exception("SerialPort::initialize_port() tcsetattr failed.");
    }
}