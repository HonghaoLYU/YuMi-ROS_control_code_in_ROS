/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef IP_CONNECTION_
#define IP_CONNECTION_

#include <config/config.hpp>

#include <vector>
#include <map>
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>

#include "visensor/visensor_datatypes.hpp"
#include "synchronization/time_synchronizer.hpp"
#include "networking/ip_data_definitions.hpp"
#include "networking/file_transfer.hpp"
#include "networking/config_connection.hpp"
#include "serial_bridge/SerialHost.hpp"
#include "sensors/sensor.hpp"

namespace visensor {

//Fpga version struct
struct FpgaConfig {
  FpgaConfig() {
    fpga_id_ = 0;
    firmware_version_major = 0;
    firmware_version_minor = 0;
    firmware_version_patch = 0;
    num_of_sensors_ = 0;
  }
  uint32_t fpga_id_;
  uint16_t firmware_version_major;
  uint16_t firmware_version_minor;
  uint16_t firmware_version_patch;
  uint16_t num_of_sensors_;
};

class IpConnection : public visensor::ConfigConnection {
 public:
  typedef boost::shared_ptr<IpConnection> Ptr;
  typedef boost::weak_ptr<IpConnection> WeakPtr;

  IpConnection();
  ~IpConnection();

  void connect(std::string hostname);
  bool sendLedConfig(uint16_t value);
  // set lightning mode:
  // 0 - strobe
  // 1 - continuous
  bool sendLedConfig(bool value);

  void sendExternalTriggerConfig(SensorId::SensorId sensor_id, const ViExternalTriggerConfig config);

  void addSensor(SensorId::SensorId sensor_id, Sensor::Ptr sensor);
  void startSensor(SensorId::SensorId sensor_id, uint32_t rate);
  void stopSensor(SensorId::SensorId sensor_id);

  bool readCameraCalibration(SensorId::SensorId sensor_id, unsigned int slot_id, ViCameraCalibration &calib_out);
  bool writeCameraCalibration(SensorId::SensorId sensor_id, unsigned int slot_id, const ViCameraCalibration calib);

  //serial bridge communication
  void registerSerialHost(SerialHost::WeakPtr serial_host);
  SerialHost::Ptr getSerialHostPointer();

  void sendSerialData(ViSerialData::Ptr data_ptr);
  void setSerialDelimiter(const char sensor_id, const std::string delimiter);
  void setSerialBaudrate(const char sensor_id, const unsigned int baudrate);

  uint32_t getId() const {
    return fpga_config_.fpga_id_;
  }

  bool fpgaInitialized() const {
    return connected_;
  }

  virtual void writeConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                           uint32_t val, uint8_t comType);
  virtual void readConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                          uint32_t &val, uint8_t comType);

  void downloadFile(std::string& local_path, std::string& remote_path);
  void uploadFile(std::string& local_path, std::string& remote_path);

  std::vector<IpComm::SensorInfo> getAttachedSensors();

 private:
  void readFpgaInfo();
  void syncTime();

  bool sendHeader(int identifier);
  void sendStartSensor(SensorId::SensorId sensor_id, uint32_t rate);
  void sendStopSensor(SensorId::SensorId sensor_id);

  template<typename T>
  bool writeDataUbi(SensorId::SensorId sensor_id, unsigned char reg, T val,
                    int numBits);
  template<typename T>
  bool readDataUbi(SensorId::SensorId sensor_id, unsigned char reg, T *val,
                   int numBits);
  bool writeDataFpga(SensorId::SensorId sensor_id, unsigned char reg, int val);
  bool readDataFpga(SensorId::SensorId sensor_id, unsigned char reg, int &val);
  bool readInitMsg(unsigned char * msg);

  uint32_t getTimestampFpgaRaw(uint8_t* buffer);
  uint64_t getTimestamp(uint8_t* buffer);

  void processPackage(SensorId::SensorId sensor_id, Measurement::Ptr new_measurement);
  void read_handler(const boost::system::error_code& e,
                    std::size_t bytes_transferred);
  void imu_read_handler(const boost::system::error_code& e,
                        std::size_t bytes_transferred);
  void serial_read_handler(const boost::system::error_code& e,
                        std::size_t bytes_transferred);

  bool receiveAck(boost::asio::ip::tcp::socket &socket);

  template<class T>
  void send_package(IpComm::Header &header, T &data);
  template<class T, class P>
  void receive_package(IpComm::Header &header, T &data,
                       P payload_type /*only used to know the datatype*/);
  template<typename T, size_t N>
  void send_payload(boost::array<T, N>& data);
  template<typename T, size_t N>
  void receive_payload(boost::asio::ip::tcp::socket &socket, boost::array<T, N>& data);

  IpComm::Header readHeader(boost::asio::ip::tcp::socket &socket);

 private:
  bool connected_;
  TimeSynchronizer time_synchronizer_;
  uint8_t number_of_sensors_;
  std::map<SensorId::SensorId, Sensor::Ptr> sensors_;
  SensorId::SensorId led_sensor_id_;  // HACK: either add the light to the cam options or create a led sensor class
  FpgaConfig fpga_config_;

  //pointer to serialhost
  SerialHost::WeakPtr serial_host_;

  // connection variables
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket data_socket_;
  boost::asio::ip::tcp::socket imu_socket_;
  boost::asio::ip::tcp::socket serial_socket_;
  boost::asio::ip::tcp::socket config_socket_;
  IpComm::HeaderPayload data_header_payload_;
  IpComm::HeaderPayload imu_header_payload_;
  IpComm::HeaderPayload serial_header_payload_;

  FileTransfer file_transfer_;
};
}  //namespace visensor

#endif /* IP_CONNECTION_ */
