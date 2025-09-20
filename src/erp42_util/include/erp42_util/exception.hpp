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
 * @file    exception.hpp
 * @brief   Exception definition
 * @author  Minkyu Kil
 * @date    2025-07-01
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_UTIL__EXCEPTION_HPP_
#define ERP42_UTIL__EXCEPTION_HPP_

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

#endif // ERP42_UTIL__EXCEPTION_HPP_