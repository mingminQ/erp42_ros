/**
 * ------------------------------------------------------------
 *
 * @file    exception.h
 * @brief   Exception definition
 * @author  Minkyu Kil
 * @date    2025-07-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * ------------------------------------------------------------
 */

#ifndef ERP42_UTILS_ECXEPTION_H_
#define ERP42_UTILS_ECXEPTION_H_

#include <stdexcept>
#include <string>

namespace erp42
{
    /** @brief Exception class inhereted STL runtime_error */
    class Exception : public std::runtime_error
    {
    // "Exeption" member functions
    public:

        /** @brief Class construcor requires error type */
        explicit Exception(const std::string &error_type) 
        : std::runtime_error(error_type)
        {
        }

        /** @brief Class construcor requires error type and additional message */
        Exception(const std::string &error_type, const std::string &message) 
        : std::runtime_error(error_type + " " + message)
        {
        }

        /** @brief Class destrucor requires no exception */
        ~Exception() noexcept override = default;

    }; // class Exception

} // namespace erp42

#endif // ERP42_UTILS_ECXEPTION_H_