#ifndef THROTTLE_COMMAND_HPP
#define THROTTLE_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing throttle command information.
         */
        struct ThrottleCommand {
            std_msgs::Header header;   /**< Standard header with metadata */
            float position;        /**< Throttle position */
            float intensity;       /**< Throttle intensity */
            bool enable;               /**< Enable or disable throttle */

            /**
             * @brief Default constructor.
             */
            ThrottleCommand()
                : header(), position(0.0f), intensity(0.0f), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param pos Throttle position.
             * @param inten Throttle intensity.
             * @param en Throttle enable status.
             */
            ThrottleCommand(const std_msgs::Header& hdr, float pos, float inten, bool en)
                : header(hdr), position(pos), intensity(inten), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // THROTTLE_COMMAND_HPP
