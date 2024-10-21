#ifndef CALIBRATION_COMMAND_HPP
#define CALIBRATION_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing calibration command information.
         */
        struct CalibrationCommand {
            std_msgs::Header header;  /**< Standard header with metadata */
            bool do_calib;            /**< Flag to start calibration */
            bool ack_calib;           /**< Acknowledgment of calibration command */

            /**
             * @brief Default constructor.
             */
            CalibrationCommand()
                : header(), do_calib(false), ack_calib(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param calib Flag to start calibration.
             * @param ack Acknowledgment of calibration command.
             */
            CalibrationCommand(const std_msgs::Header& hdr, bool calib, bool ack)
                : header(hdr), do_calib(calib), ack_calib(ack) {}
        };

    } // namespace intf
} // namespace adas

#endif // CALIBRATION_COMMAND_HPP
