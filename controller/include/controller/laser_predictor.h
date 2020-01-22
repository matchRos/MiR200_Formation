#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>
#include <tf/transform_listener.h>

class LaserPredictor{
    public:
        struct GuessedValues
        {
            int number;
            std::vector<tf::Pose> poses;            
            GuessedValues()
            {
                number=0;                
                poses.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)));
            }
        };
        
        LaserPredictor( ros::NodeHandle &nh,
                        std::string topic_name_front,
                        std::string topic_name_back);

        int getNumberOfPredictions();

        std::vector<tf::Pose> getPose();

        tf::Pose getPose(int i);

        sensor_msgs::PointCloud getRegisteredPoints();

        void guess(GuessedValues values);

    private:
        ros::NodeHandle nh_;

        tf::TransformListener listener_;
        ros::Subscriber sub_front_;
        ros::Subscriber sub_back_;

        bool first_front_msgs_received_;
        bool first_back_msgs_received_;

        std::string base_frame_;

        tf::StampedTransform trafo_front_;
        tf::StampedTransform trafo_back_;

        laser_geometry::LaserProjection projector_;

        std::vector< tf::Pose> estaminated_poses_;
        sensor_msgs::PointCloud registered_point_cloud_;
        GuessedValues guessed_values_;       


        std::vector<tf::Point> kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers); 
        std::vector<tf::Point> kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Pose> &centers); 

        void subscriberFrontCallback(const sensor_msgs::LaserScanConstPtr msg);
        void subscriberBackCallback(const sensor_msgs::LaserScanConstPtr msg);
};
