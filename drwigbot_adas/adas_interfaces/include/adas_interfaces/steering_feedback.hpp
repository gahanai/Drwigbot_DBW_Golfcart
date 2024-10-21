#ifndef STEERING_FEEDBACK_HPP
#define STEERING_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing steering feedback information.
         */
        struct SteeringFeedback {
            std_msgs::Header header;      /**< Standard header with metadata */
            int current_pos;          /**< current position */
            int target_pos;         /**< Target position of the steering */
            int current_pos_pot;    /**< Current position from the potentiometer */
            int current_pos_enc;    /**< Current position from the encoder */
            bool fault_status;            /**< Indicates if there is a fault */
            bool enable;                 /**< Indicates if the steering is enabled */

            /**
             * @brief Default constructor.
             */
            SteeringFeedback()
                : header(), target_pos(0.0f), current_pos_pot(0.0f), current_pos_enc(0.0f), fault_status(false), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param tgt_pos Target position of the steering.
             * @param curr_pos_pot Current position from the potentiometer.
             * @param curr_pos_enc Current position from the encoder.
             * @param fault Fault status.
             * @param en Enable status.
             */
            SteeringFeedback(const std_msgs::Header& hdr, float tgt_pos, float curr_pos_pot, float curr_pos_enc, bool fault, bool en)
                : header(hdr), target_pos(tgt_pos), current_pos_pot(curr_pos_pot), current_pos_enc(curr_pos_enc), fault_status(fault), enable(en) {}
        };

    } // namespace intf
} // namespace adas

#endif // STEERING_FEEDBACK_HPP
