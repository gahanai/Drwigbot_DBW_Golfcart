#ifndef BRAKE_FEEDBACK_HPP
#define BRAKE_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing brake feedback information.
         */
        struct BrakeFeedback {
            std_msgs::Header header; /**< Standard header with metadata */
            int current_pos;          /**< current position */
            int target_pos;              /**< Target position of the brake */
            int current_pos_pot;         /**< Current position from the potentiometer */
            int current_pos_enc;         /**< Current position from the encoder */
            bool fault_status;             /**< Indicates if there is a fault */
            bool enable;                   /**< Indicates if the brake is enabled */

            /**
             * @brief Default constructor.
             */
            BrakeFeedback()
                : header(), target_pos(0.0f), current_pos_pot(0.0f), current_pos_enc(0.0f), fault_status(false), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param target Target position of the brake.
             * @param pot Position from the potentiometer.
             * @param enc Position from the encoder.
             * @param fault Fault status.
             * @param en Enable status.
             */
            BrakeFeedback(const std_msgs::Header& hdr, float target, float pot, float enc, bool fault, bool en)
                : header(hdr), target_pos(target), current_pos_pot(pot), current_pos_enc(enc), fault_status(fault), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // BRAKE_FEEDBACK_HPP
