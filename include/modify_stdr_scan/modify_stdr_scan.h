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
        float max_range;
        vector<float> pt1;
        ros::Publisher modified_scan_pub;
        ros::Publisher modified_bumper_pub;
        ros::Publisher marker_ego_pub;
        ros::Publisher marker0_pub, marker1_pub, marker2_pub, marker3_pub, marker4_pub,
                       marker5_pub, marker6_pub, marker7_pub, marker8_pub, marker9_pub,
                       marker10_pub, marker11_pub, marker12_pub, marker13_pub, marker14_pub,
                       marker15_pub, marker16_pub, marker17_pub, marker18_pub, marker19_pub,
                       marker20_pub, marker21_pub, marker22_pub, marker23_pub, marker24_pub;
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
        // bool cmp(pair<string, vector<double>>& a, pair<string, vector<double>>& b);     
        vector<pair<string, vector<double> >> sort_and_prune(map<string, vector<double>>& M);

};