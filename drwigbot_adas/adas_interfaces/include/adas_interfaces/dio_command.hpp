#ifndef DIO_COMMAND_HPP
#define DIO_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing vehicle command information.
         */
        struct DioCommand {
            std_msgs::Header header;      /**< Standard header with metadata */
            bool auto_mode;               /**< Indicates if the vehicle is in auto mode */
            bool forward_direction;        /**< Indicates if the vehicle is moving forward */
            bool speed_control;            /**< Indicates if speed control is enabled */
            bool low_light;                /**< Indicates if low light mode is active */
            bool high_light;               /**< Indicates if high light mode is active */
            bool brake_light;              /**< Indicates if the brake light is on */
            bool horn;                     /**< Indicates if the horn is activated */
            bool left_indicator;           /**< Indicates if the left indicator is on */
            bool right_indicator;          /**< Indicates if the right indicator is on */
            bool enable;

            /**
             * @brief Default constructor initializing to default values.
             */
            DioCommand()
                : header(), auto_mode(false), forward_direction(false),
                  speed_control(false), low_light(false), high_light(false),
                  brake_light(false), horn(false), left_indicator(false),
                  right_indicator(false), enable(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param autoMode Indicates if the vehicle is in auto mode.
             * @param forwardDir Indicates if the vehicle is moving forward.
             * @param speedCtrl Indicates if speed control is enabled.
             * @param lowLight Indicates if low light mode is active.
             * @param highLight Indicates if high light mode is active.
             * @param brake Indicates if the brake light is on.
             * @param honk Indicates if the horn is activated.
             * @param leftInd Indicates if the left indicator is on.
             * @param rightInd Indicates if the right indicator is on.
             * @param enableVal enable all cmd if this is on.
             */
            DioCommand(const std_msgs::Header& hdr, bool autoMode, bool forwardDir,
                       bool speedCtrl, bool lowLight, bool highLight,
                       bool brake, bool honk, bool leftInd, bool rightInd, bool enableVal)
                : header(hdr), auto_mode(autoMode), forward_direction(forwardDir),
                  speed_control(speedCtrl), low_light(lowLight), high_light(highLight),
                  brake_light(brake), horn(honk), left_indicator(leftInd),
                  right_indicator(rightInd), enable(enableVal) {}

        };

    } // namespace intf
} // namespace adas

#endif // DIO_COMMAND_HPP
