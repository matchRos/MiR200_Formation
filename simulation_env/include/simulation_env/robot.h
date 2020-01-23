#include <ros/ros.h>
#include <tf/tf.h>
#include <controller/controller.h>
#include <controller/laser_predictor.h>

class Robot{
    public:
        Robot(ros::NodeHandle &nh);
            
    private:
        LaserPredictor laser_predictor_;
        Controller* controller_;
};