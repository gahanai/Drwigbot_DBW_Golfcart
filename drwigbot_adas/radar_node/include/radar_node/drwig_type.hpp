#ifndef DRWIG_TYPE_HPP
#define DRWIG_TYPE_HPP
#include <array>
#include  <cstddef>

constexpr uint32_t kDrwigDataMagicNumberSize{8U};
constexpr uint32_t kDrwigDataHeaderFrameSize{40U};
constexpr uint32_t kMaxDetObject{200U};
constexpr uint32_t kMaxClusterObject{24U};
constexpr uint32_t kMaxNumTracker{24U};

struct DrwigDataMagicNumber { 
     std::array<uint8_t,kDrwigDataMagicNumberSize> data;
};


struct DrwigDataHeader {
    uint32_t version;
    uint32_t total_packet_len;
    uint32_t platform;
    uint32_t frame_number;
    uint32_t time_cpu_cycles;
    uint32_t num_detected_obj;
    uint32_t num_tlvs;
    uint32_t sub_frame_number;
};


enum class DrwigTlvTypes : uint32_t
{
    MSG_NULL = 0,
    /*! @brief   List of detected points */
    MSG_DETECTED_POINTS,

    /*! @brief   Cluster  */
    MSG_CLUSTERS,

    /*! @brief   tracked object */
    MSG_TRACKED_OBJ,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MSG_PARKING_ASSIST ,

    /*! @brief   Range/Doppler detection matrix */
    MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MSG_STATS,

    /*! @brief   List of detected points side information */
    MSG_DETECTED_POINTS_SIDE_INFO,

    MSG_MAX
};


struct DrwigTlvHeader {
        DrwigTlvTypes type;
        uint32_t length;
};


struct DrwigObjectHeader {
    uint16_t num_obj;
    uint16_t format;
};

struct DrwigDetectedObject {
    int16_t doppler_velovity;
    uint16_t peak_value;
    int16_t x;
    int16_t y;
    int16_t z;
};

struct DrwigClusterObject {
    int16_t x;
    int16_t y;
    uint16_t width;
    uint16_t length;
};

struct DrwigTrackedObject {
    int16_t x;
    int16_t y;
    int16_t vx;
    int16_t vy;
    uint16_t width;
    uint16_t length;
};

struct DrwigDataDetectedObj {
        uint16_t   rangeIdx;     /*!< @brief Range index */
        uint16_t   dopplerIdx;   /*!< @brief Dopler index */
        uint16_t   peakVal;      /*!< @brief Peak value */
        int16_t  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
        int16_t  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
        int16_t  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
    };



#endif