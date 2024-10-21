#ifndef DRWIG_PARSER_HPP
#define DRWIG_PARSER_HPP
#include <array>
#include <libserial/SerialPort.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>
#include "ts_queue.hpp"
#include "drwig_type.hpp"

#include <bits/stdc++.h>
#include <functional>
#include <unistd.h>

using namespace LibSerial ;

using DrwigDetectObjectHandler  = std::function<void(const DrwigObjectHeader& , const std::vector<DrwigDetectedObject>& )>;
using DrwigClusterObjectHandler = std::function<void(const DrwigObjectHeader& , const std::vector<DrwigClusterObject>& )>;
using DrwigTrackedObjectHandler  = std::function<void(const DrwigObjectHeader& , const std::vector<DrwigTrackedObject>& )>;

class DrwigParser {
   
   public:
   using Frame_t = std::vector<uint8_t>;
   DrwigParser(const std::string & a_serial_port,
   int a_baud_rate);
   ~DrwigParser();
   void Parse();
   void Init();
   void ReadFrame();
   void ParseFrame();
   void Wait();
   void RegisterObjectHandler(DrwigDetectObjectHandler call_back);
   void RegisterObjectHandler(DrwigClusterObjectHandler call_back);
   void RegisterObjectHandler(DrwigTrackedObjectHandler call_back);
   private:
   void ParseData();
   void ParseDataImpl();
   void ReadUartImpl();
   void printDetectedObjData(const DrwigObjectHeader & header, const std::vector<DrwigDetectedObject>& detected_object);
   void printClusterObjData(const DrwigObjectHeader & header, const std::vector<DrwigClusterObject>& cluster_object);
   void printTrackedData(const DrwigObjectHeader & header, const std::vector<DrwigTrackedObject>& tracked_object);
   void implementBrakingLogic(); 
   void OpenComPort();
   bool isMagicNumber();
   void readMagicNumber();
   bool readHeader(Frame_t &c_frame);
   bool readTlvHeader(Frame_t &c_frame);
   bool readObjectHeader(Frame_t &c_frame, DrwigObjectHeader &obj);
   bool readDetectedObject(Frame_t &c_frame);
   bool readClusterObject(Frame_t &c_frame);
   bool readTrackedObject(Frame_t &c_frame);
   DrwigDataHeader m_drwig_data_header;
   DrwigTlvHeader m_drwg_tlv_header;
   DrwigDataMagicNumber m_magic_number;
   DrwigObjectHeader m_drwig_object_header;

   DrwigObjectHeader m_drwig_detected_object_hdr;
   DrwigObjectHeader m_drwig_cluster_object_hdr;
   DrwigObjectHeader m_drwig_tracked_object_hdr;
   
   std::vector<DrwigDetectedObject> m_drwig_detected_object;
   std::vector<DrwigClusterObject> m_drwig_cluster_object;
   std::vector<DrwigTrackedObject> m_drwig_tracked_object;
   TSQueue<uint32_t> write_queue;
   TSQueue<uint32_t> reader_queue;

   static constexpr std::array<uint8_t,kDrwigDataMagicNumberSize> kMagicNumber {2, 1, 4, 3, 6, 5, 8, 7};
   uint8_t magnumBytes[8] = {0};
   static constexpr uint32_t max_frame_size{32768};
   std::vector<Frame_t> m_buffer;
   bool read_thread;
   std::thread read_uart;
   std::thread prse_data;
   // mutex for thread synchronization 
   std::mutex enter_m_mutex;
   std::mutex buffer_m_mutex;
   uint32_t data_idx;

   std::string serial_port;
   int baud_rate;
   // Instantiate a SerialPort object.
   SerialPort m_serial_object ;
    size_t ms_timeout = 250 ;

   DrwigDetectObjectHandler drwig_detected_object_handler;
   DrwigClusterObjectHandler drwig_cluster_object_handler;
   DrwigTrackedObjectHandler drwig_tracked_object_handler;
};


#endif
