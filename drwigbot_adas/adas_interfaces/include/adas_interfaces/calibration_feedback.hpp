#ifndef CALIBRATION_FEEDBACK_HPP
#define CALIBRATION_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing calibration feedback information.
         */
        struct CalibrationFeedback {
            std_msgs::Header header;  /**< Standard header with metadata */
            uint8_t status;           /**< Calibration status */
            uint8_t fault_code;       /**< Fault code if an issue is detected */
            uint8_t stbs_status;      /**< Status of the STBS (Steering Torque Bias Sensor) */

            /**
             * @brief Default constructor.
             */
            CalibrationFeedback()
                : header(), status(0), fault_code(0), stbs_status(0) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param stat Calibration status.
             * @param fault Fault code.
             * @param stbs Status of the STBS.
             */
            CalibrationFeedback(const std_msgs::Header& hdr, uint8_t stat, uint8_t fault, uint8_t stbs)
                : header(hdr), status(stat), fault_code(fault), stbs_status(stbs) {}
        };

    } // namespace intf
} // namespace adas

#endif // CALIBRATION_FEEDBACK_HPP
