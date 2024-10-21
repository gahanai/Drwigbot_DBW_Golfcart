#ifndef DIO_FEEDBACK_HPP
#define DIO_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing DIO feedback information.
         */
        struct DioFeedback {
            std_msgs::Header header;               /**< Standard header with metadata */
            bool auto_mode_status;                  /**< Indicates if the vehicle is in auto mode */
            bool manual_mode_status;                /**< Indicates if the vehicle is in manual mode */
            bool estop_status;                      /**< Indicates if the emergency stop is activated */
            bool forward_status;                    /**< Indicates if the vehicle is in forward gear */
            bool reverse_status;                    /**< Indicates if the vehicle is in reverse gear */
            bool speed_control_status;              /**< Indicates if speed control is active */
            bool low_light_status;                  /**< Indicates if low light mode is active */
            bool high_light_status;                 /**< Indicates if high light mode is active */
            bool brake_light_status;                /**< Indicates if the brake light is on */
            bool horn_status;                       /**< Indicates if the horn is activated */
            bool left_indicator_status;             /**< Indicates if the left indicator is on */
            bool right_indicator_status;            /**< Indicates if the right indicator is on */

            /**
             * @brief Default constructor initializing to default values.
             */
            DioFeedback()
                : header(), auto_mode_status(false), manual_mode_status(false),
                  estop_status(false), forward_status(false), reverse_status(false),
                  speed_control_status(false), low_light_status(false), 
                  high_light_status(false), brake_light_status(false),
                  horn_status(false), left_indicator_status(false),
                  right_indicator_status(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param autoModeStatus Indicates if the vehicle is in auto mode.
             * @param manualModeStatus Indicates if the vehicle is in manual mode.
             * @param estopStatus Indicates if the emergency stop is activated.
             * @param forwardStatus Indicates if the vehicle is in forward gear.
             * @param reverseStatus Indicates if the vehicle is in reverse gear.
             * @param speedControlStatus Indicates if speed control is active.
             * @param lowLightStatus Indicates if low light mode is active.
             * @param highLightStatus Indicates if high light mode is active.
             * @param brakeLightStatus Indicates if the brake light is on.
             * @param hornStatus Indicates if the horn is activated.
             * @param leftIndicatorStatus Indicates if the left indicator is on.
             * @param rightIndicatorStatus Indicates if the right indicator is on.
             */
            DioFeedback(const std_msgs::Header& hdr, bool autoModeStatus, bool manualModeStatus,
                            bool estopStatus, bool forwardStatus, bool reverseStatus,
                            bool speedControlStatus, bool lowLightStatus, bool highLightStatus,
                            bool brakeLightStatus, bool hornStatus, bool leftIndicatorStatus,
                            bool rightIndicatorStatus)
                : header(hdr), auto_mode_status(autoModeStatus), manual_mode_status(manualModeStatus),
                  estop_status(estopStatus), forward_status(forwardStatus),
                  reverse_status(reverseStatus), speed_control_status(speedControlStatus),
                  low_light_status(lowLightStatus), high_light_status(highLightStatus),
                  brake_light_status(brakeLightStatus), horn_status(hornStatus),
                  left_indicator_status(leftIndicatorStatus), right_indicator_status(rightIndicatorStatus) {}
        };

    } // namespace intf
} // namespace adas

#endif // DIO_FEEDBACK_HPP
