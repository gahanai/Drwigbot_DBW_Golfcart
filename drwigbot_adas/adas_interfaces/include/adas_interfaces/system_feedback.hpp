#ifndef SYSTEM_FEEDBACK_HPP
#define SYSTEM_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {
        /**
         * @brief Struct representing system feedback information.
         */
        struct SystemFeedback {
            std_msgs::Header header;                 /**< Standard header with metadata */
            SystemStatusDBW status_dbw;                      /**< Status of Drive-by-Wire */
            bool status_sw_emergency;                /**< Status of software emergency */
            bool status_manual_override;             /**< Status of manual override */
            bool status_reset;                       /**< Status of system reset */
            bool status_steering;                    /**< Status of steering control */
            bool status_throttle;                    /**< Status of throttle control */
            bool status_brake;                       /**< Status of brake control */
            bool status_dio;                     /**< Status of dio control */
            bool status_logs;                        /**< Status of logging */
            uint8_t status_system_mode;              /**< System mode status */
            bool status_sos;                         /**< Status of SOS functionality */

            /**
             * @brief Default constructor.
             */
            SystemFeedback()
                : header(), status_dbw(SystemStatusDBW::kNone), status_sw_emergency(false), status_manual_override(false),
                  status_reset(false), status_steering(false), status_throttle(false), status_brake(false),
                  status_dio(false), status_logs(false), status_system_mode(0), status_sos(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param dbw Status of Drive-by-Wire.
             * @param sw_emergency Status of software emergency.
             * @param manual_override Status of manual override.
             * @param reset Status of system reset.
             * @param steering Status of steering control.
             * @param throttle Status of throttle control.
             * @param brake Status of brake control.
             * @param dio Status of dio control.
             * @param logs Status of logging.
             * @param system_mode Status of system mode.
             * @param sos Status of SOS functionality.
             */
            SystemFeedback(const std_msgs::Header& hdr, SystemStatusDBW dbw, bool sw_emergency, bool manual_override, bool reset,
                           bool steering, bool throttle, bool brake, bool dio, bool logs, uint8_t system_mode, bool sos)
                : header(hdr), status_dbw(dbw), status_sw_emergency(sw_emergency), status_manual_override(manual_override),
                  status_reset(reset), status_steering(steering), status_throttle(throttle), status_brake(brake),
                  status_dio(dio), status_logs(logs), status_system_mode(system_mode), status_sos(sos) {}
        };

    } // namespace intf
} // namespace adas

#endif // SYSTEM_FEEDBACK_HPP
