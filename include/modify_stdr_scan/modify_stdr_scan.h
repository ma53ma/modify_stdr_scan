#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <map>
#include <string>
#include <vector>

using namespace std;

class StdrScanModifier {
    private:
        int n;
        map<string, vector<double>> odom_map;
        float robot_radius;
        float x_obs;
        float y_obs;
        float theta_obs;
        vector<float> pt1;
        ros::Publisher modified_scan_pub;
        ros::Publisher modified_bumper_pub;
        ros::Publisher marker0_pub;
        ros::Publisher marker1_pub;
        ros::Publisher marker2_pub;
        //ros::Publisher marker3_pub;


    public:
        StdrScanModifier(int n, ros::NodeHandle nh, double _robot_radius);
        vector<ros::Subscriber> odom_subscribers;
        // void sort(map<string, vector<double>>& map);
        // bool ego_proximity(pair<string, vector<double>>& a,pair<string, vector<double>>& b);
        void ego_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void other_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void publish_marker(ros::Publisher publisher, const nav_msgs::Odometry::ConstPtr& msg, float color[]);
        int sgn_star(float dy);
        void ego_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};