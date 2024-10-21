#ifndef BATTERY_FEEDBACK_H
#define BATTERY_FEEDBACK_H

#include <cstdint>  // For uint8_t
#include "common.hpp"   // For std_msgs::Header
namespace adas {
    namespace intf {

/**
 * @brief Struct representing battery feedback information.
 */
struct BatteryFeedback {
    std_msgs::Header header;         /**< Standard header with metadata */
    float current_charge;            /**< Current battery charge */
    float discharge_rate;            /**< Rate at which the battery is discharging */
    bool mode;                       /**< Battery mode (e.g., normal, eco) */
    bool fault;                      /**< Flag indicating if there's a fault */
    uint8_t fault_code;              /**< Code representing the type of fault */

    /**
     * @brief Default constructor.
     */
    BatteryFeedback()
        : header(), current_charge(0.0f), discharge_rate(0.0f), mode(false), fault(false), fault_code(0) {}

    /**
     * @brief Parameterized constructor.
     * @param hdr Header information.
     * @param charge Current battery charge.
     * @param rate Discharge rate.
     * @param mode Battery mode.
     * @param fault Flag indicating if there's a fault.
     * @param code Fault code.
     */
    BatteryFeedback(const std_msgs::Header& hdr, float charge, float rate, bool mode, bool fault, uint8_t code)
        : header(hdr), current_charge(charge), discharge_rate(rate), mode(mode), fault(fault), fault_code(code) {}
};
    } // namespace intf
} // namespace adas
#endif // BATTERY_FEEDBACK_H
