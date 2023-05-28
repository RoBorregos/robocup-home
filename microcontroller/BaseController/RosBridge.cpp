#include "RosBridge.h"

// velocity_subscriber_("/cmd_vel",&RosBridge::velocityCallback, this)
// encoder_publisher_("/base_control/encoders", &encoders_msg_)

//////////////////////////////////Constructor//////////////////////////////////////
RosBridge::RosBridge(Movement *move_all, BNO *bno, Plot *plot) : move_all_(move_all), plot_(plot), bno_(bno) {
    
    // Timers
    odom_timer_ = millis();
    watchdog_timer_ = millis();
    
    // Message Init
    time_delta = 0;
    back_left_encoders = 0;
    back_right_encoders = 0;
    front_left_encoders = 0;
    front_right_encoders = 0;

    for(int i = 0; i < kCountMotors; ++i) {
		  last_encoder_counts_[i] = 0;
    }
}

//////////////////////////////////Velocity Suscriber//////////////////////////////////////
void RosBridge::velocityCallback(double linearx, double lineary, double angularz) {
    linearX_ = linearx;
    linearY_ = lineary;
    angularZ_ = angularz;
    watchdog_timer_ = millis();
}

//////////////////////////////////Encoders Publisher//////////////////////////////////////
void RosBridge::getEncoderCounts() {
	int new_encoder_counts[kCountMotors];
  new_encoder_counts[0] = move_all_->back_left_motor_.getOdomTicks();
  new_encoder_counts[1] = move_all_->back_right_motor_.getOdomTicks();
  new_encoder_counts[2] = move_all_->front_left_motor_.getOdomTicks();
  new_encoder_counts[3] = move_all_->front_right_motor_.getOdomTicks();
	// find deltas
	int delta_encoder_counts[kCountMotors];
	for(int i = 0; i < kCountMotors; ++i) {
		int count_diff = new_encoder_counts[i] - last_encoder_counts_[i];
    delta_encoder_counts[i] = (count_diff + kCountMax/2) % kCountMax - kCountMax/2;
    
    // reset delta if it exceeds threshold
    if(abs(delta_encoder_counts[i]) > kCountReset) {
        delta_encoder_counts[i] = 0;
    }
    
    // update previous count
    last_encoder_counts_[i] = new_encoder_counts[i];
	}
    back_left_encoders = delta_encoder_counts[0];
    back_right_encoders = delta_encoder_counts[1];
    front_left_encoders = delta_encoder_counts[2];
    front_right_encoders = delta_encoder_counts[3];
}

void RosBridge::executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer) {
  switch (command) {
    case 0x04: // Velocity command
      if (packet_size == 13) { // Check packet size
        float x, y, angular;
        memcpy(&x, buffer, sizeof(x));
        memcpy(&y, buffer + sizeof(x), sizeof(y));
        memcpy(&angular, buffer + sizeof(x) + sizeof(y), sizeof(angular));
        velocityCallback(x, y, angular);
        writeSerial(true, nullptr, 0);
      }
      break;
    case 0x13: // Hardware Version 
      if (packet_size == 1) { // Check packet size
        uint32_t version[] = {1};
        writeSerial(true, (uint8_t*)version, sizeof(version));
      }
      break;
    case 0x00: // Baud
      if (packet_size == 1) { // Check packet size
        uint32_t baud[] = {57600};
        writeSerial(true, (uint8_t*)baud, sizeof(baud));
      }
      break;
    case 0x02: // Get Encoders
      if (packet_size == 1) { // Check packet size
        getEncoderCounts();
        int data[] = {back_left_encoders, back_right_encoders, front_left_encoders, front_right_encoders};
        writeSerial(true, (uint8_t*)data, sizeof(data));
      }
      break;
    case 0x41: // Reset IMU
      if (packet_size == 1) { // Check packet size
        writeSerial(true, nullptr, 0);
        bno_->reset();
      }
      break;
    case 0x05: // Get IMU
      if (packet_size == 1) { // Check packet size
        float data[] = {bno_->getYaw(), bno_->getYawVel(), bno_->getXAccel(), bno_->getYAccel(), bno_->getZAccel()};
        writeSerial(true, (uint8_t*)data, sizeof(data));
      }
      break;
    case 0x15: // Get Emergency Button
      if (packet_size == 1) { // Check packet size
          uint8_t status[] = {digitalRead(emergency_btn_pin)}; // Read the button status
          writeSerial(true, (uint8_t*)status, sizeof(status));
      }
      break;
    case 0x03: // Reset Encoders
      if (packet_size == 1) { // Check packet size
        move_all_->back_left_motor_.setOdomTicks(0);
        move_all_->back_right_motor_.setOdomTicks(0);
        move_all_->front_left_motor_.setOdomTicks(0);
        move_all_->front_right_motor_.setOdomTicks(0);
        writeSerial(true, nullptr, 0);
      }
      break;
    default:
      break;
  }
}


void RosBridge::writeSerial(bool success, uint8_t* payload, int elements) {
  uint8_t ack = success ? 0x00 : 0x01;
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(((int)sizeof(uint8_t)) * elements + 1); // Packet size
  Serial.write(ack); // ACK
  
  // Send payload bytes
  for (size_t i = 0; i < elements; i++) {
    Serial.write(payload[i]);
  }
  Serial.write(0x00); // Footer
  Serial.flush();

}
void RosBridge::readSerial() {
  static uint8_t buffer[20];
  static uint8_t index = 0;
  static uint8_t packet_size = 0;
  static uint8_t command = 0;
  static uint8_t check_sum = 0;
  
  while (Serial.available()) {
    buffer[index++] = Serial.read();
    // Check packet header
    if (index == 1 && buffer[0] != 0xFF) {
      index = 0;
      packet_size = 0;
      command = 0;
    }
    if (index == 2 && buffer[1] != 0xAA) {
      packet_size = 0;
      command = 0;
      index = 0;
    }
    
    // Read packet size and command
    if (index == 4) {
      packet_size = buffer[2];
      command = buffer[3];
    }
    
    // Check if the entire packet has been received
    if (index == 3 + (packet_size) + 1) {
      check_sum = buffer[index - 1];
      if (check_sum != command + 1) {
        // Checksum error
        index = 0;
        packet_size = 0;
        command = 0;
        continue;
      }
      // Execute the command
      executeCommand(packet_size, command, &buffer[4]);
      
      // Reset index and packet_size
      index = 0;
      packet_size = 0;
    }
  }
}

//////////////////////////////////Run//////////////////////////////////////
void RosBridge::run() {
    while(1) {
        readSerial();
        if((millis() - watchdog_timer_) > kWatchdogPeriod) {
            linearX_ = 0.0;
            linearY_ = 0.0;
            angularZ_ = 0.0;        
            watchdog_timer_ = millis();
        }
        move_all_->cmdVelocity(linearX_, linearY_, angularZ_);
        bno_->updateBNO();
    }
}
