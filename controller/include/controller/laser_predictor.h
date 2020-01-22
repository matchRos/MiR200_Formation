#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>

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
        
        LaserPredictor(ros::NodeHandle &nh);

        int getNumberOfPredictions();

        std::vector<tf::Pose> getPose();

        tf::Pose getPose(int i);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        laser_geometry::LaserProjection projector_;

        std::vector< tf::Pose> estaminated_poses_;
        sensor_msgs::PointCloud registered_point_cloud_;
        GuessedValues guessed_values_;       


        std::vector<sensor_msgs::PointCloud> kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers); 
        std::vector<sensor_msgs::PointCloud> kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Pose> &centers); 

        void subscriberCallback(const sensor_msgs::LaserScanPtr msg);
};
