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

#ifndef IMU_ADIS16488_HPP_
#define IMU_ADIS16488_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "sensors/imu.hpp"

namespace visensor
{
namespace ImuAdis16488Defaults
{
  const int
  RATE=IMU_FREQUENCY,
  MSG_SIZE=(7*4+4*2),
  NUM_OF_MSGS_IN_PACKAGE=10,
  CALIBRATION_SIZE=0;
  const bool USE_CONST_PACKAGE_SIZE=true;
}

class ImuAdis16488 : public Imu {
 public:
  typedef boost::shared_ptr<ImuAdis16488> Ptr;

  ImuAdis16488(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
  virtual ~ImuAdis16488(){};

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();

 private:
  void allocateBuffer();
  int getNumOfNewMsgs();
  void resetBuffer();
  void getGyro(uint8_t * buffer, double * gyro);
  void getAcc(uint8_t * buffer, double * acc);
  void getMag(uint8_t * buffer, double * mag);
  void getBaro(uint8_t * buffer, double * baro);
  void getTemp(uint8_t * buffer, double * temp);
  void newImuMsgs();
  uint64_t getTimestamp();
};
} //namespace visensor

#endif /* IMU_ADIS16488_HPP_ */
