#include <modify_stdr_scan/modify_stdr_scan.h>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

using namespace std;

StdrScanModifier::StdrScanModifier(int n, ros::NodeHandle nh, double _robot_radius) {
    n = n;
    robot_radius = _robot_radius; // this is really the obstacle radius
    x_obs = 0.0f;
    y_obs = 0.0f;
    theta_obs = 0.0f;
    pt1.push_back(x_obs);
    pt1.push_back(y_obs);

    max_range = 5.0;

    modified_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/robot" + to_string(n) + "/mod_laser_0", 1);
    modified_bumper_pub = nh.advertise<sensor_msgs::Range>("/robot" + to_string(n) + "/mod_bumpers", 1); 
    marker_ego_pub = nh.advertise<visualization_msgs::Marker>("/robot" + to_string(n) + "/marker", 1);
    marker0_pub = nh.advertise<visualization_msgs::Marker>("/robot0/marker", 1);
    marker1_pub = nh.advertise<visualization_msgs::Marker>("/robot1/marker", 1);
    marker2_pub = nh.advertise<visualization_msgs::Marker>("/robot2/marker", 1);
    marker3_pub = nh.advertise<visualization_msgs::Marker>("/robot3/marker", 1);
    marker4_pub = nh.advertise<visualization_msgs::Marker>("/robot4/marker", 1);
    marker5_pub = nh.advertise<visualization_msgs::Marker>("/robot5/marker", 1);
    marker6_pub = nh.advertise<visualization_msgs::Marker>("/robot6/marker", 1);
    marker7_pub = nh.advertise<visualization_msgs::Marker>("/robot7/marker", 1);
    marker8_pub = nh.advertise<visualization_msgs::Marker>("/robot8/marker", 1);
    marker9_pub = nh.advertise<visualization_msgs::Marker>("/robot9/marker", 1);
    marker10_pub = nh.advertise<visualization_msgs::Marker>("/robot10/marker", 1);
    marker11_pub = nh.advertise<visualization_msgs::Marker>("/robot11/marker", 1);
    marker12_pub = nh.advertise<visualization_msgs::Marker>("/robot12/marker", 1);
    marker13_pub = nh.advertise<visualization_msgs::Marker>("/robot13/marker", 1);
    marker14_pub = nh.advertise<visualization_msgs::Marker>("/robot14/marker", 1);
    marker15_pub = nh.advertise<visualization_msgs::Marker>("/robot15/marker", 1);
    marker16_pub = nh.advertise<visualization_msgs::Marker>("/robot16/marker", 1);
    marker17_pub = nh.advertise<visualization_msgs::Marker>("/robot17/marker", 1);
    marker18_pub = nh.advertise<visualization_msgs::Marker>("/robot18/marker", 1);
    marker19_pub = nh.advertise<visualization_msgs::Marker>("/robot19/marker", 1);
    marker20_pub = nh.advertise<visualization_msgs::Marker>("/robot20/marker", 1);
    marker21_pub = nh.advertise<visualization_msgs::Marker>("/robot21/marker", 1);
    marker22_pub = nh.advertise<visualization_msgs::Marker>("/robot22marker", 1);
    marker23_pub = nh.advertise<visualization_msgs::Marker>("/robot23/marker", 1);
    marker24_pub = nh.advertise<visualization_msgs::Marker>("/robot24/marker", 1);    
}

void StdrScanModifier::ego_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // cout << "in ego odom cb" << endl;
    x_obs = msg->pose.pose.position.x;
    y_obs = msg->pose.pose.position.y;
    geometry_msgs::Quaternion q = msg->pose.pose.orientation;

    ros::NodeHandle nh;

    theta_obs = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));    
    pt1.at(0) = x_obs;
    pt1.at(1) = y_obs;
    // cout << "test 1" << endl;

    float color[3] = {0.0, 0.0, 1.0};
    publish_marker(marker_ego_pub, msg, color);
}

void StdrScanModifier::other_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // cout << "in other odom cb" << endl;
    vector<double> pos{msg->pose.pose.position.x,msg->pose.pose.position.y};
    string key = msg->child_frame_id + "/odom";
    // cout << key << endl;
    odom_map[key] = pos;

    
    
    sensor_msgs::Range range = sensor_msgs::Range();
    if (sqrt(pow(pt1.at(0)-pos.at(0), 2) + pow(pt1.at(1)-pos.at(1), 2)) < robot_radius*2) {
        range.range = -std::numeric_limits<double>::infinity();
    } else {
        range.range = std::numeric_limits<double>::infinity();
    }
    modified_bumper_pub.publish(range);
    
    //ros::NodeHandle nh;
    float color[3] = {1.0, 0.0, 0.0};
    string check = msg->child_frame_id;
    // cout << "check " << check << endl;
    
    if (check == "robot0") { publish_marker(marker0_pub, msg, color);
    } else if (check == "robot1") { publish_marker(marker1_pub, msg, color);
    } else if (check == "robot2") { publish_marker(marker2_pub, msg, color);
    } else if (check == "robot3") { publish_marker(marker3_pub, msg, color);
    } else if (check == "robot4") { publish_marker(marker4_pub, msg, color);
    } else if (check == "robot5") { publish_marker(marker5_pub, msg, color);
    } else if (check == "robot6") { publish_marker(marker6_pub, msg, color);
    } else if (check == "robot7") { publish_marker(marker7_pub, msg, color);
    } else if (check == "robot8") { publish_marker(marker8_pub, msg, color);
    } else if (check == "robot9") { publish_marker(marker9_pub, msg, color);
    } else if (check == "robot10") { publish_marker(marker10_pub, msg, color);
    } else if (check == "robot11") { publish_marker(marker11_pub, msg, color);
    } else if (check == "robot12") { publish_marker(marker12_pub, msg, color);
    } else if (check == "robot13") { publish_marker(marker13_pub, msg, color);
    } else if (check == "robot14") { publish_marker(marker14_pub, msg, color);
    } else if (check == "robot15") { publish_marker(marker15_pub, msg, color);
    } else if (check == "robot16") { publish_marker(marker16_pub, msg, color);
    } else if (check == "robot17") { publish_marker(marker17_pub, msg, color);
    } else if (check == "robot18") { publish_marker(marker18_pub, msg, color);
    } else if (check == "robot19") { publish_marker(marker19_pub, msg, color);
    } else if (check == "robot20") { publish_marker(marker20_pub, msg, color);
    } else if (check == "robot21") { publish_marker(marker21_pub, msg, color);
    } else if (check == "robot22") { publish_marker(marker22_pub, msg, color);
    } else if (check == "robot23") { publish_marker(marker23_pub, msg, color);
    } else if (check == "robot24") { publish_marker(marker24_pub, msg, color);
    }    
}

void StdrScanModifier::publish_marker(ros::Publisher publisher, const nav_msgs::Odometry::ConstPtr& msg, float color[]) {
    // cout << "in publish marker" << endl;
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.ns = "robot_position_markers";
    marker.id = 0;

    marker.type = marker.CYLINDER;
    marker.action = marker.ADD;
    marker.pose.orientation.w = 1;

    marker.pose.position.x = msg->pose.pose.position.x;
    marker.pose.position.y = msg->pose.pose.position.y;
    marker.pose.position.z = msg->pose.pose.position.z;
    ros::Duration d(0.0);
    marker.lifetime = d;
    marker.scale.x = 2 * robot_radius;
    marker.scale.y = 2 * robot_radius;
    marker.scale.z = 0.000001;
    marker.color.a = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    // cout << "pre publisher" << endl;
    //cout << "marker publish" << endl;
    publisher.publish(marker);
    // cout << "post publisher" << endl;
}

int StdrScanModifier::sgn_star(float dy) {
    if (dy < 0) {
        return -1;
    } else {
        return 1;
    }
}

double l2_norm(vector<double> const& u) {
    double accum = 0.;
    for (int i = 0; i < u.size(); ++i) {
        accum += u[i] * u[i];
    }
    return sqrt(accum);
}

struct Comparator {
    Comparator(vector<float> _pt1) {this->pt1 = _pt1;}

    bool operator() (pair<string, vector<double>>& a, pair<string, vector<double>>& b) {
        double a_dist = pow(pt1[0] - a.second[0], 2) + pow(pt1[1] - a.second[1], 2);
        double b_dist = pow(pt1[0] - b.second[0], 2) + pow(pt1[1] - b.second[1], 2);
        return a_dist < b_dist;
    }

    vector<float> pt1;
};

vector<pair<string, vector<double> >> StdrScanModifier::sort_and_prune(map<string, vector<double>>& M)
{
  
    // Declare vector of pairs
    vector<pair<string, vector<double> >> A;
    
    // Copy key-value pair from Map
    // to vector of pairs
    double dist;
    for (auto& it : M) {
        // ROS_INFO_STREAM(it.first);
        //ROS_INFO_STREAM("pose: " << std::to_string(it.second[0]) << ", " << std::to_string(it.second[1]));
        //ROS_INFO_STREAM("ego pose: " << pt1[0] << ", " << pt1[1]);
        dist = sqrt(pow(pt1[0] - it.second[0], 2) + pow(pt1[1] - it.second[1], 2));
        //ROS_INFO_STREAM("dist: " << dist);
        if (dist < max_range) {
            A.push_back(it);
        }
    }
    
    // Sort using comparator function
    sort(A.begin(), A.end(), Comparator(pt1));
    
    // ROS_INFO_STREAM("printing pruned vect");
    // Print the sorted value
    /*
    for (auto& it : A) {
  
        ROS_INFO_STREAM(it.first << ' ' << it.second[0] << ", " << it.second[1]);
    }
    */
    

    return A;
}
 
void StdrScanModifier::ego_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // double start_time = ros::Time::now().toSec();
    sensor_msgs::LaserScan modified_laser_scan;
    modified_laser_scan.header = msg->header;
    modified_laser_scan.angle_min = msg->angle_min;
    modified_laser_scan.angle_max = msg->angle_max;
    modified_laser_scan.angle_increment = msg->angle_increment;
    modified_laser_scan.range_min = msg->range_min;
    modified_laser_scan.range_max = msg->range_max;
    modified_laser_scan.ranges = msg->ranges;

    for (int i = 0; i < modified_laser_scan.ranges.size(); i++) {
        float rad = msg->angle_min + i*msg->angle_increment;
        float dist;
        // cout << "i: " << i << " rad: " << rad << endl;
        // cout << "original distance " << modified_laser_scan.ranges[i] << endl;
        if (isinf(modified_laser_scan.ranges[i])) {
            dist = msg->range_max;
        } else {
            dist = modified_laser_scan.ranges[i];
        }

        vector<float> lidar_range{dist*cos(rad + theta_obs), dist*sin(rad + theta_obs)};

        vector<float> pt2{pt1[0] + lidar_range[0],
                          pt1[1] + lidar_range[1]};

        // map<string, vector<double>>::iterator it;
        vector<pair<string, vector<double> >> odom_vect = sort_and_prune(odom_map);
        // TODO: sort map here according to distance from robot. Then, can break after first intersection
        for (int j = 0; j < odom_vect.size(); j++) {
            vector<double> other_state = odom_vect[j].second;
            // ROS_INFO_STREAM("ODOM MAP SECOND: " << other_state[0] << ", " << other_state[1]);
            // int idx_dist = std::distance(odom_map.begin(), it);
            // ROS_INFO_STREAM("EGO ROBOT ODOM: " << pt1[0] << ", " << pt1[1]);
            // ROS_INFO_STREAM("ODOM VECT SECOND: " << other_state[0] << ", " << other_state[1]);

            vector<double> centered_pt1{pt1[0] - other_state[0],
                                        pt1[1] - other_state[1]};

            vector<double> centered_pt2{pt2[0] - other_state[0],
                                        pt2[1] - other_state[1]};
            // ROS_INFO_STREAM("LASER SCAN PT: " << pt2[0] << ", " << pt2[1]);

            double dx = centered_pt2[0] - centered_pt1[0];
            double dy = centered_pt2[1] - centered_pt1[1];
            vector<double> dx_dy{dx, dy};
            double dr = l2_norm(dx_dy);

            double D = centered_pt1[0]*centered_pt2[1] - centered_pt2[0]*centered_pt1[1];
            double discriminant = pow(robot_radius,2) * pow(dr, 2) - pow(D, 2);

            if (discriminant > 0) {
                vector<double> intersection0{(D*dy + sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                             (-D * dx + abs(dy)*sqrt(discriminant)) / pow(dr, 2)};
                                    
                vector<double> intersection1{(D*dy - sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                             (-D * dx - abs(dy)*sqrt(discriminant)) / pow(dr, 2)};
                vector<double> int0_min_cent_pt1{intersection0[0] - centered_pt1[0],
                                                 intersection0[1] - centered_pt1[1]};
                double dist0 = l2_norm(int0_min_cent_pt1);
                vector<double> int1_min_cent_pt1{intersection1[0] - centered_pt1[0],
                                                 intersection1[1] - centered_pt1[1]};
                double dist1 = l2_norm(int1_min_cent_pt1);
                vector<double> cent_pt2_min_cent_pt1{centered_pt2[0] - centered_pt1[0],
                                                     centered_pt2[1] - centered_pt1[1]};
                if (dist0 < dist1) {
                    vector<double> int0_min_cent_pt2{intersection0[0] - centered_pt2[0],
                                                     intersection0[1] - centered_pt2[1]};
                    if (dist0 < modified_laser_scan.ranges[i] && dist0 < l2_norm(cent_pt2_min_cent_pt1) && l2_norm(int0_min_cent_pt2) < l2_norm(cent_pt2_min_cent_pt1)) {
                        // ROS_INFO_STREAM("changed distance from " << modified_laser_scan.ranges[i] << " to " << dist0);
                        modified_laser_scan.ranges[i] = dist0;
                        break;
                    }
                } else {
                    vector<double> int1_min_cent_pt2{intersection1[0] - centered_pt2[0], intersection1[1] - centered_pt2[1]};
                    if (dist1 < modified_laser_scan.ranges[i] && dist1 < l2_norm(cent_pt2_min_cent_pt1) && l2_norm(int1_min_cent_pt2) < l2_norm(cent_pt2_min_cent_pt1)) {
                        // ROS_INFO_STREAM("changed distance from " << modified_laser_scan.ranges[i] << " to " << dist1);                        
                        modified_laser_scan.ranges[i] = dist1;
                        break;
                    }
                }
            }
        }
        
    }

    // ROS_INFO_STREAM("time taken: " << ros::Time::now().toSec() - start_time);
    // cout << "modified scan publish" << endl;
    modified_scan_pub.publish(modified_laser_scan);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "modify_stdr_scan");

    ros::NodeHandle nh;

    std::string robot_namespace = std::string(argv[1]);
    double robot_radius = std::stod(std::string(argv[2]));
    // std::cout << "robot_radius: " << robot_radius << std::endl;
    robot_namespace.erase(0,5); // removing "robot"
    // std::cout << "robot_namespace: " << robot_namespace << std::endl;
    char *robot_id = strdup(robot_namespace.c_str());
    int n = std::atoi(robot_id); // getting the last character in the namespace which is the id of the robot we want
    StdrScanModifier modifier = StdrScanModifier(n, nh, robot_radius); 

    // std::cout << "n: " << n << std::endl;

    for (int i = 0; i < n; i++) {
        ros::Subscriber temp_odom_sub = nh.subscribe("/robot" + to_string(i) + "/odom", 10, &StdrScanModifier::other_odom_callback, &modifier);
        modifier.odom_subscribers.push_back(temp_odom_sub);
    }

    // cout << "setting up subscribers" << endl;

    ros::Subscriber pose_sub = nh.subscribe("/robot" + to_string(n) + "/laser_0", 10, &StdrScanModifier::ego_scan_callback, &modifier);
    ros::Subscriber ego_odom_sub = nh.subscribe("/robot" + to_string(n) + "/odom", 10, &StdrScanModifier::ego_odom_callback, &modifier);
    
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}