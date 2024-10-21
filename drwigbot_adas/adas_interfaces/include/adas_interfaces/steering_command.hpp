#ifndef STEERING_COMMAND_HPP
#define STEERING_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing steering command information.
         */
        struct SteeringCommand {
            std_msgs::Header header;    /**< Standard header with metadata */
            float position;         /**< Position of the steering */
            float intensity;        /**< Intensity of the steering command */
            bool enable;                /**< Enable or disable the steering command */

            /**
             * @brief Default constructor.
             */
            SteeringCommand()
                : header(), position(0.0f), intensity(0.0f), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param pos Position of the steering.
             * @param intns Intensity of the steering command.
             * @param en Enable or disable the steering command.
             */
            SteeringCommand(const std_msgs::Header& hdr, float pos, float intns, bool en)
                : header(hdr), position(pos), intensity(intns), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // STEERING_COMMAND_HPP
