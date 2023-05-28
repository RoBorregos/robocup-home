// This class has all the functions related to the ROS connection. It receives the velocity
// commands and publish the encoders ticks for Odometry.
#ifndef RosBridge_h
#define RosBridge_h

#include <stdint.h>
#include <math.h>
#include "BNO.h"
#include <Arduino.h>

#include "Movement.h"
#include "Plot.h"

inline int sign(int a) { return min(1, max(-1, a)); };

class RosBridge{
    public:
        //////////////////////////////////Constructor//////////////////////////////////////
        RosBridge(Movement *move_all, BNO *bno, Plot *plot);
        
        
        //////////////////////////////////Run//////////////////////////////////////
        // Calls publish and verify it is still receiving velocity commands.
        void run();

    private:
        //////////////////////////////////Velocity Suscriber//////////////////////////////////////
        // Receives velocity commands.
        void velocityCallback(double linearx, double lineary, double angularz);

        //////////////////////////////////Encoders Publisher//////////////////////////////////////
        // Process encoder and return message.
        void getEncoderCounts();

        // Get Emergency Btn.
        void getEmergencyBtn();

        // Set Cmd Velocity.
        void setCmdVelocity();
        
        // Get Encoders.
        void getEncoders();

        void readSerial();

        Movement *move_all_;
        BNO *bno_;

        static constexpr uint8_t kCountMotors = 4;
        
        // Plot.
        Plot *plot_;

        // Suscriber.
        static constexpr uint16_t kWatchdogPeriod = 500;
        
        // Publisher.
        int back_left_encoders = 0.0;
        int back_right_encoders = 0.0;
        int front_left_encoders = 0.0;
        int front_right_encoders = 0.0;
        float time_delta = 0.0;
        int emergency_btn_pin = 46;

        int last_encoder_counts_[kCountMotors];
        static constexpr uint8_t kOdomPeriod = 40;
        static constexpr uint16_t kIntMax = 65535;
        static constexpr uint16_t kCountReset = 250;
        static constexpr uint16_t kCountMax = 600;
        static constexpr uint16_t kCountOverflow = 16374;        

        // Timers.
        unsigned long odom_timer_ = 0;
        unsigned long watchdog_timer_ = 0;

        // CMD Velocity.
        double linearX_ = 0;
        double linearY_ = 0;
        double angularZ_ = 0;

        void executeCommand(uint8_t packet_size, uint8_t command, uint8_t* buffer);
        void writeSerial(bool success, uint8_t* payload, int elements);

        // Debug
        bool debug_ = false;
};


#endif
