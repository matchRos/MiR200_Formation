#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>
#include <tf/transform_listener.h>

class LaserPredictor{
    public:
        struct Frames{
            Frames(std::string base,std::string front, std::string back):   base(base),
                                                                            front(front),
                                                                            back(back)
            {}
            std::string base;
            std::string front;
            std::string back;
        };
        
        
        struct Transforms{
            tf::StampedTransform front;
            tf::StampedTransform back;
        };

        struct NoPredicitonException : public std::exception {
            const char * what () const throw () 
            {
                std::stringstream ss;
                return "No Prediction was possible!";
            }
        };
        
        typedef std::vector<tf::Pose> Poses;
        typedef std::vector<tf::Point> Points;
        typedef Frames Topics;

        LaserPredictor( ros::NodeHandle &nh,
                        Frames frames,
                        Topics topic);

        int getNumberOfPredictions();

        Poses getPose();

        tf::Pose getPose(int i);

        sensor_msgs::PointCloud getRegisteredPoints();
        sensor_msgs::PointCloud getClusteredPoints();

        void guess(Poses values);

        void startClustering(double frequenzy);

    private:
        ros::NodeHandle nh_;

        tf::TransformListener listener_;

        ros::Subscriber sub_front_;
        ros::Subscriber sub_back_;

        ros::Timer cluster_scope_;

        Transforms trafos_;
        Frames frames_;

        laser_geometry::LaserProjection projector_;   

        sensor_msgs::PointCloud front_data_;
        sensor_msgs::PointCloud back_data_;
        sensor_msgs::PointCloud point_cloud_;

        sensor_msgs::PointCloud clustered_point_cloud;
        Poses poses_;

        sensor_msgs::PointCloud combineData(sensor_msgs::PointCloud front, sensor_msgs::PointCloud back);
        void transformCloud(sensor_msgs::PointCloud &cloud, tf::Transform trafo);

        void clustering(const ros::TimerEvent& event);
        std::vector<tf::Point> kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers); 
        std::vector<tf::Point> kMeans(sensor_msgs::PointCloud &data,Poses &centers); 

        void subscriberFrontCallback(const sensor_msgs::LaserScanConstPtr msg);
        void subscriberBackCallback(const sensor_msgs::LaserScanConstPtr msg);
};
