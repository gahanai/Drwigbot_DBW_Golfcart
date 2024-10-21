#ifndef VEHICLE_FEEDBACK_HPP
#define VEHICLE_FEEDBACK_HPP

#include <cstdint>      // For uint8_t
#include <string>       // For std::string
#include "common.hpp"   // For std_msgs::Header

namespace adas {
    namespace intf {

        /**
         * @brief Struct representing vehicle feedback information.
         */
        struct VehicleFeedback {
            std_msgs::Header header;                  /**< Standard header with metadata */
            float vehicle_speed_kmph;             /**< Vehicle speed in kilometers per hour */
            float vehicle_speed_mph;              /**< Vehicle speed in miles per hour */
            float vehicle_fuel_level;             /**< Vehicle fuel level */
            float vehicle_engine_rpm;             /**< Vehicle engine RPM */
            float vehicle_engine_load;            /**< Vehicle engine load */
            float vehicle_throttle_position;      /**< Vehicle throttle position */
            bool vehicle_fault;                      /**< Indicates if there is a vehicle fault */
            std::string vehicle_fault_status;        /**< Description of the vehicle fault status */
            bool ignition_status;                    /**< Ignition status of the vehicle */
            std::string shifter_status;              /**< Status of the shifter */

            /**
             * @brief Default constructor.
             */
            VehicleFeedback()
                : header(), vehicle_speed_kmph(0.0f), vehicle_speed_mph(0.0f), vehicle_fuel_level(0.0f),
                  vehicle_engine_rpm(0.0f), vehicle_engine_load(0.0f), vehicle_throttle_position(0.0f),
                  vehicle_fault(false), vehicle_fault_status(""), ignition_status(false), shifter_status("") {}

            /**
             * @brief Parameterized constructor.
             * @param hdr Header information.
             * @param speed_kmph Vehicle speed in km/h.
             * @param speed_mph Vehicle speed in mph.
             * @param fuel_level Vehicle fuel level.
             * @param engine_rpm Vehicle engine RPM.
             * @param engine_load Vehicle engine load.
             * @param throttle_pos Vehicle throttle position.
             * @param fault Vehicle fault status.
             * @param fault_status Description of the vehicle fault status.
             * @param ignition Ignition status.
             * @param shifter Status of the shifter.
             */
            VehicleFeedback(const std_msgs::Header& hdr, float speed_kmph, float speed_mph, 
                            float fuel_level, float engine_rpm, float engine_load, 
                            float throttle_pos, bool fault, const std::string& fault_status, 
                            bool ignition, const std::string& shifter_status)
                : header(hdr), vehicle_speed_kmph(speed_kmph), vehicle_speed_mph(speed_mph), 
                  vehicle_fuel_level(fuel_level), vehicle_engine_rpm(engine_rpm), vehicle_engine_load(engine_load),
                  vehicle_throttle_position(throttle_pos), vehicle_fault(fault), vehicle_fault_status(fault_status),
                  ignition_status(ignition), shifter_status(shifter_status) {}
        };

    } // namespace intf
} // namespace adas

#endif // VEHICLE_FEEDBACK_HPP
