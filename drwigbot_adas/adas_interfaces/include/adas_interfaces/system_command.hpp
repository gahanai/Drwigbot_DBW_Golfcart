#ifndef SYSTEM_COMMAND_HPP
#define SYSTEM_COMMAND_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing system command information.
         */
        struct SystemCommand {
            std_msgs::Header header;                 /**< Standard header with metadata */
            bool enable_dbw;                         /**< Enable Drive-by-Wire */
            bool enable_sw_emergency;                /**< Enable software emergency */
            bool enable_manual_override;             /**< Enable manual override */
            bool enable_reset;                       /**< Enable system reset */
            bool enable_steering;                    /**< Enable steering control */
            bool enable_throttle;                    /**< Enable throttle control */
            bool enable_brake;                       /**< Enable brake control */
            bool enable_dio;                     /**< Enable dio control */
            bool enable_logs;                        /**< Enable logging */
            uint8_t enable_system_mode;              /**< System mode */
            bool enable_sos;                         /**< Enable SOS functionality */

            /**
             * @brief Default constructor.
             */
            SystemCommand()
                : header(), enable_dbw(false), enable_sw_emergency(false), enable_manual_override(false),
                  enable_reset(false), enable_steering(false), enable_throttle(false), enable_brake(false),
                  enable_dio(false), enable_logs(false), enable_system_mode(0), enable_sos(false) {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param dbw Enable Drive-by-Wire.
             * @param sw_emergency Enable software emergency.
             * @param manual_override Enable manual override.
             * @param reset Enable system reset.
             * @param steering Enable steering control.
             * @param throttle Enable throttle control.
             * @param brake Enable brake control.
             * @param dio Enable dio control.
             * @param logs Enable logging.
             * @param system_mode System mode.
             * @param sos Enable SOS functionality.
             */
            SystemCommand(const std_msgs::Header& hdr, bool dbw, bool sw_emergency, bool manual_override, bool reset,
                          bool steering, bool throttle, bool brake, bool dio, bool logs, uint8_t system_mode, bool sos)
                : header(hdr), enable_dbw(dbw), enable_sw_emergency(sw_emergency), enable_manual_override(manual_override),
                  enable_reset(reset), enable_steering(steering), enable_throttle(throttle), enable_brake(brake),
                  enable_dio(dio), enable_logs(logs), enable_system_mode(system_mode), enable_sos(sos) {}
        };

    } // namespace intf
} // namespace adas

#endif // SYSTEM_COMMAND_HPP
