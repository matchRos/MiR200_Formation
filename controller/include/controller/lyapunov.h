#include<controller/controller.h>

class LyapunovController:public Controller{
    public:
        LyapunovController(ros::NodeHandle nh);
        ~LyapunovController();
        struct Parameter
        {
            float kx;           /**< Control gain in x-direction */
            float ky;           /**< Control gain in y-direction */ 
            float kphi;       /**< Control gain in theta-direction */
        };
        struct Velocities
        {
            double omega;
            double vd;
        };
        void load();
        Velocities calculate(tf::Transform difference,Velocities desired);
        void calculate();

    private:
        Parameter param_;
        Velocities desired_;
        Velocities output_;
}