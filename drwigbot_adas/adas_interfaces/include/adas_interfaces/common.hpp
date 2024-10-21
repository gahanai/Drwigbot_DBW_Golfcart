#ifndef COMMON_HPP
#define COMMON_HPP

#include <cstdint>   // For uint32_t
#include <string>    // For std::string

namespace std_msgs {
    /**
     * @brief Header structure for message metadata.
     */
    struct Header {
        uint32_t seq;         /**< Sequence number */
        double stamp;         /**< Timestamp */
        std::string frame_id; /**< Frame ID */

        /**
         * @brief Default constructor.
         */
        Header()
            : seq(0), stamp(0.0), frame_id("") {}

        /**
         * @brief Parameterized constructor.
         * @param sequence Sequence number.
         * @param timestamp Timestamp.
         * @param frame Frame ID.
         */
        Header(uint32_t sequence, double timestamp, const std::string& frame)
            : seq(sequence), stamp(timestamp), frame_id(frame) {}
    };
} // namespace std_msgs



namespace adas{
    namespace intf {


        /*(uint8: 0 - auto-calib in progress, 1 -
         autocalib finished, 2. DBW ready,, 3. Fault , 
         4. sw_emergency, 5. hw_emergency, 7. warning, 
         8. Ready, 9 DBW Disable) */
        enum class SystemStatusDBW : uint8_t {
            kNone=0,
            kAutoCalibInProgress=1,
            kAutoCalibFinished=2,
            kDBWReady=3,
            kFault=4,
            kSWEmergency=5,
            kHWEmergency=6,
            kWarning=7,
            kReady=8,
            kDBWDisable=9,
        };
   
         /*
         status_system_mode 
         (uint8: 0 - wait for input, 
         1 - joy stick mode, 
         2 - AV mode)
         */
        enum class SystemStatusMode : uint8_t {
            kNone=0,
            kWaitForInput=1,
            kJoyStickMode=2,
            kAVMode=3,
        };
    }
}
#endif // COMMON_HPP
