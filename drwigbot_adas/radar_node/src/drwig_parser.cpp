#include "radar_node/drwig_parser.hpp"  
DrwigParser::DrwigParser(const std::string & a_serial_port,
   int a_baud_rate):
serial_port{a_serial_port},
baud_rate{a_baud_rate},
m_buffer(10,Frame_t(max_frame_size)),
m_drwig_detected_object(kMaxDetObject),
m_drwig_cluster_object(kMaxClusterObject),
m_drwig_tracked_object(kMaxNumTracker),
drwig_detected_object_handler{nullptr},
drwig_cluster_object_handler{nullptr},
drwig_tracked_object_handler{nullptr}{

  // allocate the memory and shuffle
  // Max Possible Byte 32000
  // have credit of memory 
  for(uint32_t i=0; i < this->m_buffer.size();++i){
        write_queue.push(i);
  }

}

DrwigParser::~DrwigParser(){
     std::cout<<"\n Close DrwigParser";
    this->read_thread = false;
    if(this->read_uart.joinable()){
        this->read_uart.join();
    }
    if(this->prse_data.joinable()){
        this->prse_data.join();
    }
}
void DrwigParser::Init(){
    this->OpenComPort();
    buffer_m_mutex.lock();
}
void DrwigParser::OpenComPort(){

    try {
        m_serial_object.Open(this->serial_port);
    }
    catch (const OpenFailed&) {
        // sleep for 10 some time and open again 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        try {
            m_serial_object.Open(this->serial_port);
        }
        catch (const OpenFailed&) {
            std::cerr << "\n[DrwigParser]- Error : Serial Port Open Failed "
             <<this->serial_port<<" Baudrate "<<
             this->baud_rate;
            std::_Exit(EXIT_FAILURE);
        }
    }
    // if(m_serial_object.isOpen()){
    //     this->m_serial_object = serial_object;
    //     read_thread = true;
    //     std::cout<<"\n[DrwigParser]- Serial Open Completed"
    // }

    // Set the baud rate of the serial port.
    m_serial_object.SetBaudRate(BaudRate::BAUD_921600) ;

    // Set the number of data bits.
    m_serial_object.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    m_serial_object.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    m_serial_object.SetParity(Parity::PARITY_NONE) ;
    
    // Set the number of stop bits.
    m_serial_object.SetStopBits(StopBits::STOP_BITS_1) ;

        // Wait for data to be available at the serial port.
    while(!m_serial_object.IsDataAvailable()) 
    {
        usleep(1000) ;
    }
    this->read_thread = true;
}

bool DrwigParser::isMagicNumber(){
    bool status{true};
    if(this->m_magic_number.data.size() != kMagicNumber.size()){
        std::cerr << "\n[DrwigParser]- Magic Number Size miss match";
        std::_Exit(EXIT_FAILURE);
    }
    for(uint32_t i{0U}; i < this->m_magic_number.data.size() ; i++)
    {
       if( this->m_magic_number.data[i] != kMagicNumber[i])
       {
          status = false;
          break;
       }
    }    
    return status;
    
}
void DrwigParser::readMagicNumber(){

    this->m_magic_number.data.fill(0);
    while(!isMagicNumber())
    {
      for(uint32_t iter{0}; iter < 
        (this->m_magic_number.data.size() - 1);
        ++iter){
            this->m_magic_number.data[iter] = 
            this->m_magic_number.data[iter+1];
        }
    // Read one byte from the serial port and print it to the terminal.
        
        uint8_t data_byte ;
        try
        {
            // Read a single byte of data from the serial port.
            m_serial_object.ReadByte(data_byte, ms_timeout) ;

        }
        catch (const ReadTimeout&)
        {
            std::cerr << "\nThe ReadByte() call has timed out." << std::endl ;
            continue;
        }
        this->m_magic_number.
        data[
            this->m_magic_number.data.size() - 1
            ] = data_byte;  
    }
}

 void DrwigParser::ReadUartImpl(){

    this->readMagicNumber();
        this->m_magic_number.data.fill(0);
            while(this->read_thread){
                if(write_queue.IsEmpty()){
                    buffer_m_mutex.lock();
                }
                uint32_t idx{write_queue.pop()};
                Frame_t &c_frame{m_buffer.at(idx)};
                c_frame.clear();
                while(!isMagicNumber())
                {
                    for(uint32_t iter{0}; iter < 
                        (this->m_magic_number.data.size() - 1);
                        ++iter){
                            this->m_magic_number.data[iter] = 
                            this->m_magic_number.data[iter+1];
                        }

                        // Read one byte from the serial port and print it to the terminal.
                        uint8_t data_byte ;
                        try
                        {
                            // Read a single byte of data from the serial port.
                            m_serial_object.ReadByte(data_byte, ms_timeout) ;

                        }
                        catch (const ReadTimeout&)
                        {
                            std::cerr << "\nThe ReadByte() call has timed out." << std::endl ;
                            continue;
                        }
                       this->m_magic_number.
                        data[
                            this->m_magic_number.data.size() - 1
                            ] = data_byte;
                        c_frame.push_back(data_byte);  
                }
                // magic number found 
                if(c_frame.size() > this->m_magic_number.data.size()){
                    c_frame.resize( c_frame.size()- this->m_magic_number.data.size());
                }
                this->m_magic_number.data.fill(0);
                this->reader_queue.push(idx);
                buffer_m_mutex.unlock();    
            }
        this->enter_m_mutex.unlock();
 }


void DrwigParser::ParseFrame() {
    if(reader_queue.IsEmpty()){
            buffer_m_mutex.lock();
    }
    uint32_t idx{reader_queue.pop()};
    Frame_t &c_frame{m_buffer.at(idx)};
    // parse the header 
    this->m_drwig_detected_object.resize(0);
    this->m_drwig_cluster_object.resize(0);
    this->m_drwig_tracked_object.resize(0);
    if(this->readHeader(c_frame)){ 
         for(uint32_t tlv_idx{0U}; tlv_idx< this->m_drwig_data_header.num_tlvs;++tlv_idx){
            if(this->readTlvHeader(c_frame)) {
                switch (this->m_drwg_tlv_header.type){
                    case DrwigTlvTypes::MSG_DETECTED_POINTS:{
                        this->readDetectedObject(c_frame);
                        if(this->drwig_detected_object_handler!=nullptr){
                             this->drwig_detected_object_handler(m_drwig_detected_object_hdr, m_drwig_detected_object);   
                        }
                        break;
                    }
                     case DrwigTlvTypes::MSG_CLUSTERS:{
                        this->readClusterObject(c_frame);
                        if(this->drwig_cluster_object_handler!=nullptr){
                              this->drwig_cluster_object_handler(m_drwig_cluster_object_hdr, m_drwig_cluster_object);   
                        }
                        break;
                    }
                     case DrwigTlvTypes::MSG_TRACKED_OBJ:{
                        this->readTrackedObject(c_frame);
                        if(this->drwig_tracked_object_handler!=nullptr){
                              this->drwig_tracked_object_handler(m_drwig_tracked_object_hdr, m_drwig_tracked_object);   
                        }
                        // Implement braking logic
                        implementBrakingLogic();
                        break;
                    }
                  default:{
                    // Skip the TLV Data
                    this->data_idx += this->m_drwg_tlv_header.length;
                    break;
                  }
                }          
              }          
           }
       }
       if(write_queue.IsEmpty()){
          write_queue.push(idx);
          buffer_m_mutex.unlock();
          std::cout<<"\n UnLock buffer_m_mutex 3";
       }
       else {
        write_queue.push(idx);
       }
}

void DrwigParser::implementBrakingLogic() {
    const float roi_x_limit = 1.5; // X position limit
    const float roi_range_limit = 5.0; // Range limit
    const float ttc_threshold = 2.0; // Time to collision threshold

    int nearest_object_index = -1;
    float nearest_object_distance = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < m_drwig_tracked_object.size(); ++i) {
        float x_pos = static_cast<float>(m_drwig_tracked_object[i].x) / pow(2, m_drwig_tracked_object_hdr.format);
        float y_pos = static_cast<float>(m_drwig_tracked_object[i].y) / pow(2, m_drwig_tracked_object_hdr.format);
        float vx = static_cast<float>(m_drwig_tracked_object[i].vx) / pow(2, m_drwig_tracked_object_hdr.format);
        float vy = static_cast<float>(m_drwig_tracked_object[i].vy) / pow(2, m_drwig_tracked_object_hdr.format);
        float range = sqrt(x_pos * x_pos + y_pos * y_pos);
        float doppler_velocity = (x_pos * vx + y_pos * vy) / range;

        // Check if the object is within ROI
        if (fabs(x_pos) <= roi_x_limit && range <= roi_range_limit) {
            std::cout << "Object within ROI for tracked obj " << i << ": X: " << x_pos << ", Range: " << range << std::endl;

            // Check for the nearest object
            if (range < nearest_object_distance) {
                nearest_object_distance = range;
                nearest_object_index = i;
            }
        }
    }

    // Check if we have a nearest object
    if (nearest_object_index != -1) {
        float nearest_range = nearest_object_distance;
        float nearest_doppler_velocity = (static_cast<float>(m_drwig_tracked_object[nearest_object_index].vx) +
                                           static_cast<float>(m_drwig_tracked_object[nearest_object_index].vy)) / 
                                           pow(2, m_drwig_tracked_object_hdr.format);

        std::cout << "Nearest Object Range: " << nearest_range << ", Doppler Velocity: " << nearest_doppler_velocity << std::endl;

        // Calculate Time to Collision (TTC)
        if (nearest_doppler_velocity < 0) {  
            float ttc = nearest_range / fabs(nearest_doppler_velocity);
            std::cout << "Nearest Object Range: " << nearest_range << ", Doppler Velocity: " << nearest_doppler_velocity 
                      << ", TTC: " << ttc << " seconds" << std::endl;

            // Check TTC condition
            if (ttc < ttc_threshold) {
                std::cout << "Brake ON" << std::endl;  
            } else {
                std::cout << "No impending collision detected (object moving away or stationary)." << std::endl;
            }
        } else {
            std::cout << "No impending collision detected (object moving away or stationary)." << std::endl;
        }
    }
}

void DrwigParser::ParseDataImpl() {
    while(this->read_thread){
        this->ParseFrame();
    
    }
}

void DrwigParser::ParseData(){
    std::thread threadWorker(&DrwigParser::ParseDataImpl, this);
    this->prse_data = std::move(threadWorker);

}

void DrwigParser::ReadFrame(){
    // sync first
    this->enter_m_mutex.lock();
    std::thread threadWorker(&DrwigParser::ReadUartImpl, this);
    this->read_uart = std::move(threadWorker);

}
void DrwigParser::Wait(){
    if(this->read_uart.joinable()){
        this->read_uart.join();
    }
    if(this->prse_data.joinable()){
        this->prse_data.join();
    }
}
void DrwigParser::Parse(){
    this->ReadFrame();
    this->ParseData();
    this->Wait();
}

bool DrwigParser::readTlvHeader(Frame_t &c_frame){
    bool status{false};
    if(c_frame.size() >  (this->data_idx + sizeof(DrwigTlvHeader))){
        std::memcpy(&this->m_drwg_tlv_header, &c_frame.at(this->data_idx), sizeof(DrwigDataHeader));
            status = true;
            this->data_idx += sizeof(DrwigTlvHeader);
    }
    return status;
}
bool DrwigParser::readHeader(Frame_t &c_frame){
    this->data_idx = 0;
    bool status{false};
    if(c_frame.size() > sizeof(DrwigDataHeader)){
        std::memcpy(&this->m_drwig_data_header, c_frame.data(), sizeof(DrwigDataHeader));
        status = true;
        this->data_idx += sizeof(DrwigDataHeader);
    }
  return status;
}

bool DrwigParser::readObjectHeader(Frame_t &c_frame, DrwigObjectHeader &obj){
   bool status{false};
    if(c_frame.size() > (this->data_idx + sizeof(DrwigObjectHeader))){
        std::memcpy(&obj, &c_frame.at(this->data_idx), sizeof(DrwigObjectHeader));
        this->data_idx += sizeof(DrwigObjectHeader);
        status = true;
    }
  return status;
}

bool DrwigParser::readDetectedObject(Frame_t &c_frame){
    bool status{false};
    uint32_t idx = this->data_idx;
    // read the header
    DrwigObjectHeader obj_hdr;
    if(this->readObjectHeader(c_frame,obj_hdr)){
       this->m_drwig_detected_object.resize(obj_hdr.num_obj);
       std::memcpy(this->m_drwig_detected_object.data(), &c_frame.at(this->data_idx), obj_hdr.num_obj*sizeof(DrwigDetectedObject));
       this->data_idx += (obj_hdr.num_obj*sizeof(DrwigDetectedObject));
       this->m_drwig_detected_object_hdr = obj_hdr;
       status = true;
    }
   return status; 
}
bool DrwigParser::readClusterObject(Frame_t &c_frame){
    bool status{false};
    DrwigObjectHeader obj_hdr;
    if(this->readObjectHeader(c_frame,obj_hdr)){
        this->m_drwig_cluster_object.resize(obj_hdr.num_obj);
        std::memcpy(this->m_drwig_cluster_object.data(), &c_frame.at(this->data_idx), obj_hdr.num_obj*sizeof(DrwigClusterObject));
        this->data_idx += (obj_hdr.num_obj*sizeof(DrwigClusterObject));
        this->m_drwig_cluster_object_hdr = obj_hdr;
        status = true;
    }
   return status; 
}
bool DrwigParser::readTrackedObject(Frame_t &c_frame){
    bool status{false};
    DrwigObjectHeader obj_hdr;
    if(this->readObjectHeader(c_frame,obj_hdr)){
        this->m_drwig_tracked_object.resize(obj_hdr.num_obj);
        std::memcpy(this->m_drwig_tracked_object.data(), &c_frame.at(this->data_idx), obj_hdr.num_obj*sizeof(DrwigTrackedObject));
        this->data_idx += (obj_hdr.num_obj*sizeof(DrwigTrackedObject));
        this->m_drwig_tracked_object_hdr = obj_hdr;
        status = true;
    }
    return status;
}
void DrwigParser::RegisterObjectHandler(DrwigDetectObjectHandler call_back){
    this->drwig_detected_object_handler = call_back;
}
void DrwigParser::RegisterObjectHandler(DrwigClusterObjectHandler call_back){
    this->drwig_cluster_object_handler = call_back;

}
void DrwigParser::RegisterObjectHandler(DrwigTrackedObjectHandler call_back){
    this->drwig_tracked_object_handler = call_back;

}

void DrwigParser::printDetectedObjData(const DrwigObjectHeader & header, const std::vector<DrwigDetectedObject>& detected_object){
   //std::cout<<"\nTotal Number of Detected Object "<<header.num_obj;
   double result = pow(2, header.format);
   for(uint32_t i{0U};i<detected_object.size();i++){
    float x = static_cast<float>(detected_object[i].x)/result;
    float y = static_cast<float>(detected_object[i].y)/result;
    float z = static_cast<float>(detected_object[i].z)/result;
    float range = sqrt(x*x + y*y + z*z);
    std::cout<<"\n No."<< i <<" : range "<< range<<" , x "<< x <<",y "<<y<<",z :"<<z 
    <<" , doppler_velovity : "<<detected_object[i].doppler_velovity<<
    " peak_value "<<detected_object[i].peak_value;
   }
}


void DrwigParser::printClusterObjData(const DrwigObjectHeader & header, const std::vector<DrwigClusterObject>& cluster_object){
      std::cout<<"\nTotal Number of Cluster Object "<<header.num_obj;
      double result = pow(2, header.format);
        for(uint32_t i{0U};i<cluster_object.size();i++){
            float x_center = static_cast<float>(cluster_object[i].x)/result;
            float y_center = static_cast<float>(cluster_object[i].y)/result;
            float cluster_width = static_cast<float>(cluster_object[i].width)/result;
            float cluster_length = static_cast<float>(cluster_object[i].length)/result;
            std::cout<<"\n No."<< i <<"-x_center "<< x_center <<",y_center "<<y_center<<",cluster_width :"<<cluster_width 
            <<" , cluster_length : "<<cluster_length;
        }
}
void DrwigParser::printTrackedData(const DrwigObjectHeader & header, const std::vector<DrwigTrackedObject>& tracked_object){
   std::cout<<"\nTotal Number of Tracked Object "<<header.num_obj;
   double result = pow(2, header.format);
   for(uint32_t i{0U};i<tracked_object.size();i++){
    float x_pos = static_cast<float>(tracked_object[i].x)/result;
    float y_pos = static_cast<float>(tracked_object[i].y)/result;
    float vx = static_cast<float>(tracked_object[i].vx)/result;
    float vy = static_cast<float>(tracked_object[i].vy)/result;
    float width = static_cast<float>(tracked_object[i].width)/result;
    float length = static_cast<float>(tracked_object[i].length)/result;
    std::cout<<"\n No."<< i <<"-x_pos "<< x_pos <<",y_pos "<<y_pos
       <<"-vx "<< vx <<",vy "<<vy<<",width :"<<width 
    <<" , length : "<<length;
   }
}
