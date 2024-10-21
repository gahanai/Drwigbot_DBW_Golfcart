#ifndef BRAKE_COMMAND_HPP
#define BRAKE_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing brake command information.
         */
        struct BrakeCommand {
            std_msgs::Header header;   /**< Standard header with metadata */
            float position;           /**< Position of the brake */
            float intensity;          /**< Intensity of the brake command */
            bool enable;              /**< Enable or disable the brake */

            /**
             * @brief Default constructor.
             */
            BrakeCommand()
                : header(), position(0.0f), intensity(0.0f), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param pos Brake position.
             * @param inten Brake intensity.
             * @param en Enable status.
             */
            BrakeCommand(const std_msgs::Header& hdr, float pos, float inten, bool en)
                : header(hdr), position(pos), intensity(inten), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // BRAKE_COMMAND_HPP
