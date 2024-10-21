#ifndef THROTTLE_FEEDBACK_HPP
#define THROTTLE_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing throttle feedback information.
         */
        struct ThrottleFeedback {
            std_msgs::Header header;           /**< Standard header with metadata */
            int target_pos;              /**< Target position of the throttle */
            int current_pos;          /**< current position */
            int current_pos_pot;         /**< Current position from potentiometer */
            int current_pos_enc;         /**< Current position from encoder */
            bool fault_status;                 /**< Fault status indicator */
            bool enable;                       /**< Enable status of the throttle */

            /**
             * @brief Default constructor.
             */
            ThrottleFeedback()
                : header(), target_pos(0.0f), current_pos_pot(0.0f), current_pos_enc(0.0f), fault_status(false), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param target Target position of the throttle.
             * @param pos_pot Current position from potentiometer.
             * @param pos_enc Current position from encoder.
             * @param fault Fault status.
             * @param en Enable status.
             */
            ThrottleFeedback(const std_msgs::Header& hdr, float target, float pos_pot, float pos_enc, bool fault, bool en)
                : header(hdr), target_pos(target), current_pos_pot(pos_pot), current_pos_enc(pos_enc), fault_status(fault), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // THROTTLE_FEEDBACK_HPP
