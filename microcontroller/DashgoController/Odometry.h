// This class has all the functions related to the ROS connection. It receives the velocity
// commands and publish the encoders ticks for Odometry.
#ifndef Odometry_h
#define Odometry_h

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <base_control/StampedEncoders.h>
#include <base_control/Encoders.h>

#include <stdint.h>
#include <math.h>
#include <Arduino.h>

#include "Movement.h"
#include "Plot.h"

inline int sign(int a) { return min(1, max(-1, a)); };

enum class OdometryState {
    Linear = 1, 
    Angular = 2, 
    Stop = 3
};

class Odometry{
    public:
        //////////////////////////////////Constructor//////////////////////////////////////
        Odometry(Movement *move_all,ros::NodeHandle *nh, Plot *plot);
        
        
        //////////////////////////////////Run//////////////////////////////////////
        // Calls publish and verify it is still receiving velocity commands.
        void run();

    private:
        //////////////////////////////////Velocity Suscriber//////////////////////////////////////
        // Receives velocity commands.
        void velocityCallback(const geometry_msgs::Twist& cmdvel);
        
        // Process velocity commands.
        void cmdVelocity(const double linearx, const double lineary,  const double angularz);
        
        //////////////////////////////////Encoders Publisher//////////////////////////////////////
        // Process encoder and return message.
        void getEncoderCounts();
        
        // Publish encoder message.
        void publish();
        
        Movement *move_all_;
        OdometryState odom_state_ = OdometryState::Stop;
        static constexpr uint8_t kCountMotors = 2;

        // Node.
        ros::NodeHandle  *nh_;
        
        // Plot.
        Plot *plot_;

        // Suscriber.
        ros::Subscriber<geometry_msgs::Twist, Odometry> velocity_subscriber_;
        static constexpr uint16_t kWatchdogPeriod = 500;
        
        // Publisher.
        ros::Publisher encoder_publisher_;
        base_control::StampedEncoders encoders_msg_;
        int last_encoder_counts_[kCountMotors];
        static constexpr uint8_t kOdomPeriod = 40;
        static constexpr uint16_t kIntMax = 65535;
        static constexpr uint16_t kCountReset = 250;
        static constexpr uint16_t kCountOverflow = 16374;        

        // Timers.
        unsigned long odom_timer_ = 0;
        unsigned long watchdog_timer_ = 0;

        // CMD Velocity.
        double linearX_ = 0;
        double linearY_ = 0;
        double angularZ_ = 0;
        
};


#endif
