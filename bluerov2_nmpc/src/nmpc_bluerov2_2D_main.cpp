/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   Jan 2023
 *
 */

#include <nmpc_bluerov2_2D_main.h>
#include <tf/transform_datatypes.h> // Include tf for tf::Point
#include <unordered_map>
#include <optional> // For std::optional
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace ros;
double sampleTime = 0.02;

double distx;
double disty;
double distz;
double current_ref_x;
double current_ref_y;
double current_ref_z;

double test_x;
double test_y;

std::vector<double> ref_traj_x;
std::vector<double> ref_traj_y;
std::vector<double> ref_traj_z;

std::vector<double> neptune_planner_x;
std::vector<double> neptune_planner_y;
std::vector<double> neptune_planner_z;

int size_refs = 10;
int neptune_planner_size = 10;

int count_traj = 0;
int neptune_planner_count = 0;
std::vector<double> closest_obs;
double a, b, c;

std::vector<std::vector<double>> obstacle_centers;
std::vector<std::vector<double>> path;
std::vector<geometry_msgs::Point> tether_positions; // Vector to store tether positions

// Define a small tolerance for floating-point comparison
const double TOLERANCE = 1e-6;
double neptune_x;
double neptune_y;

bool CONTACT = 0;

bool arePointsEqual(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return (std::fabs(p1.x - p2.x) < TOLERANCE && std::fabs(p1.y - p2.y) < TOLERANCE);
}

geometry_msgs::Point *getFirstContactPoint(const std::vector<geometry_msgs::Point> &tether)
{
    std::unordered_map<std::string, int> pointCount;

    for (const auto &point : tether)
    {
        std::string key = std::to_string(point.x) + "_" + std::to_string(point.y);
        // std::cout << "Processing point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
        // std::cout << "Generated key: " << key << std::endl;

        // Check if this point has appeared before based on its (x, y) values
        pointCount[key]++;

        if (pointCount[key] == 2)
        {
            // Print the contact point
            std::cout << "First contact point found: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            return new geometry_msgs::Point(point); // Dynamically allocate to return a pointer
        }
    }

    return nullptr; // Return nullptr if no contact point is found
}

/**/
void neptuneTetherCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    // Check if the points array has elements
    // tether_positions.clear();
    tether_positions.resize(msg->points.size());

    if (!msg->points.empty())
    {
        // Iterate through all points in the message and add them to tether_positions
        // for (const auto& point : msg->points) {
        //    tether_positions.push_back(point);  // Add each point to the vector
        // }
        for (int i = 0; i < msg->points.size(); i++)
        {

            tether_positions[i] = msg->points[i];
        }

        // Print the size of the points array in red
        std::cout << "\033[31mSize of points array: " << msg->points.size() << "\033[0m" << std::endl;

        // Optionally, print all points added to tether_positions
        std::cout << "Tether positions updated with " << msg->points.size() << " new points." << std::endl;

        // Optional: Print each point in the tether_positions
        for (const auto &point : tether_positions)
        {
            std::cout << "Tether Position: x = " << point.x
                      << ", y = " << point.y
                      << ", z = " << point.z << std::endl;
        }
    }
}

// Function to find the quadratic polynomial coefficients a, b, and c for y = ax^2 + bx + c
void findQuadraticPolynomial(const std::vector<double> &p1, const std::vector<double> &p2, double slope, double &a, double &b, double &c)
{
    // Extract coordinates
    double x1 = p1[0], y1 = p1[1];
    double x2 = p2[0], y2 = p2[1];

    // Solve for a and b using the given points and slope condition
    // System of equations:
    // 1. y1 = a*x1^2 + b*x1 + c
    // 2. y2 = a*x2^2 + b*x2 + c
    // 3. 2*a*x1 + b = slope

    double coeffA = (y2 - y1) - slope * (x2 - x1);
    double coeffC = (x2 * x2 - x1 * x1);

    // Ensure we are not dividing by zero
    if (coeffC == 0)
    {
        throw std::runtime_error("The points must not be the same.");
    }

    // Calculate 'a' using the system of equations
    a = coeffA / coeffC;

    // Calculate 'b' using the slope condition
    b = slope - 2 * a * x1;

    // Calculate 'c' using the first point's equation
    c = y1 - a * x1 * x1 - b * x1;
}

// Function to find the perpendicular slope of a vector
double calculatePerpendicularSlope(const std::vector<double> &vector)
{
    if (vector[1] == 0)
    {
        throw std::invalid_argument("Vertical vector: perpendicular slope is undefined");
    }
    return -vector[0] / vector[1];
}

std::vector<std::vector<double>> CalculatePathPolynomial(const std::vector<double> &rov_pos,
                                                         const geometry_msgs::Point &contact_point,
                                                         const std::vector<double> &obstacle_center,
                                                         int n)
{
    // a) Find vector OC
    std::vector<double> OC = {contact_point.x - obstacle_center[0], contact_point.y - obstacle_center[1]};

    // b) Find the perpendicular slope to OC
    double slope = calculatePerpendicularSlope(OC);

    // c) Find the quadratic polynomial coefficients a, b, c
    double a, b, c;
    std::vector<double> p1(2);
    p1[0] = contact_point.x;
    p1[1] = contact_point.y;

    findQuadraticPolynomial(p1, rov_pos, slope, a, b, c);

    // Output the results
    std::cout << "Quadratic polynomial: y = " << a << "x^2 + " << b << "x + " << c << std::endl;
    std::cout << "Perpendicular slope: " << slope << std::endl;

    // d) Generate the path points along the quadratic path
    std::vector<std::vector<double>> path_points;
    double x_step = (rov_pos[0] - contact_point.x) / (n - 1); // Divide the range into n points
    for (int i = 0; i < n; ++i)
    {
        double x = contact_point.x + i * x_step; // Calculate x position for each point
        double y = a * x * x + b * x + c;        // Calculate corresponding y using the quadratic polynomial
        path_points.push_back({x, y});           // Store the point in the vector
    }

    return path_points;
}

// Callback to handle incoming obstacle data
void obstacles_cb(const decomp_ros_msgs::PolyhedronArray::ConstPtr &msg)
{
    obstacle_centers.clear();

    for (const auto &polyhedron : msg->polyhedrons)
    {
        double cx = 0.0, cy = 0.0, cz = 0.0;
        int n_vertices = polyhedron.points.size();

        // Calculate the center by averaging the vertices
        for (const auto &vertex : polyhedron.points)
        {
            cx += vertex.x;
            cy += vertex.y;
            cz += vertex.z;
        }
        cx /= n_vertices;
        cy /= n_vertices;
        cz /= n_vertices;

        // Store the center of this polyhedron
        obstacle_centers.push_back({cx, cy, cz});
    }

    ROS_INFO("Received %lu obstacles", obstacle_centers.size());
}

// Function to generate tether points linearly spaced between base and rov_pos
/*
void generate_tether_points_linear(std::vector<double> base, std::vector<double> rov_pos, int N_points, std::vector<std::vector<double>> &tether_points)
{
    // Calculate the difference in each axis
    double dx = (rov_pos[0] - base[0]) / (N_points - 1);
    double dy = (rov_pos[1] - base[1]) / (N_points - 1);
    double dz = (rov_pos[2] - base[2]) / (N_points - 1);

    // Add base as the starting point
    tether_points.push_back(base);

    // Generate intermediate points
    for (int i = 1; i < N_points - 1; i++)
    {
        std::vector<double> point(3);
        point[0] = base[0] + i * dx;
        point[1] = base[1] + i * dy;
        point[2] = base[2] + i * dz;
        tether_points.push_back(point);
    }

    // Add rov_pos as the final point
    tether_points.push_back(rov_pos);
}

*/

// Function to find the closest obstacle to the ROV position
std::vector<double> findClosestObstacle(const geometry_msgs::Point *conct_pos, std::vector<double> rov_position)
{
    double min_distance = std::numeric_limits<double>::max();
    std::vector<double> closest_obstacle_center;
    for (const auto &center : obstacle_centers)
    {
        double distance;
        if (conct_pos != nullptr)
        {
            distance = std::sqrt(
                std::pow(center[0] - conct_pos->x, 2) +
                std::pow(center[1] - conct_pos->y, 2) +
                std::pow(center[2] - conct_pos->z, 2));
        }
        else
        {
            // conct_pos.x is empty
            // Handle the empty case here if needed
            distance = std::sqrt(
                std::pow(center[0] - rov_position[0], 2) +
                std::pow(center[1] - rov_position[1], 2) +
                std::pow(center[2] - rov_position[2], 2));
        }

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_obstacle_center = center;
        }
    }

    ROS_INFO("Closest obstacle at (%f, %f, %f) with distance %f",
             closest_obstacle_center[0], closest_obstacle_center[1],
             closest_obstacle_center[2], min_distance);

    return closest_obstacle_center;
}

// Neptune subscriber:
void neptune_planner_traj_cb(const nav_msgs::Path::ConstPtr &msg)
{
    neptune_planner_count = 0;

    neptune_planner_size = msg->poses.size();

    for (const auto &pose_stamped : msg->poses)
    {
        // Extract position data and push it to the respective trajectory vectors
        neptune_planner_x.push_back(pose_stamped.pose.position.x);
        neptune_planner_y.push_back(pose_stamped.pose.position.y);
        neptune_planner_z.push_back(pose_stamped.pose.position.z);
    }
}

/*
void neptune_state_cb(const snapstack_msgs::State::ConstPtr& msg)
{
    // Access position fields using msg->pos.x, msg->pos.y, and msg->pos.z
    ROS_INFO("Position: x=%f, y=%f, z=%f", msg->pos.x, msg->pos.y, msg->pos.z);
}
*/

void ref_traj_cb(const nav_msgs::Path::ConstPtr& msg)
{
    count_traj = 0;



    size_refs =msg->poses.size();

   cout<< "SIZE"<< size_refs<<endl;

   for (const auto& pose_stamped : msg->poses) {
        // Extract position data and push it to the respective trajectory vectors
    ref_traj_x.push_back(pose_stamped.pose.position.x);
       ref_traj_y.push_back(pose_stamped.pose.position.y);
        ref_traj_z.push_back(pose_stamped.pose.position.z);
    }
      cout<< " ref_traj_x"<<  ref_traj_x[0]<<endl;

    ROS_INFO("Filled trajectory vectors with %zu points", ref_traj_x.size());

// Using back() to get the most recent point
if (!msg->poses.empty()) {
    test_x = msg->poses.back().pose.position.x;
    test_y = msg->poses.back().pose.position.y;
    ROS_INFO("Most recent point: x = %f, y = %f", test_x, test_y);
} else {
    ROS_WARN("Path message is empty.");
}


}


void initialize_reference_if_Empty()
{
    // Check if current_ref_x is NaN and assign the first element of ref_path_x if it is
    if (std::isnan(current_ref_x) && !ref_traj_x.empty())
    {
        current_ref_x = ref_traj_x[0];
    }

    // Check if current_ref_y is NaN and assign the first element of ref_path_y if it is
    if (std::isnan(current_ref_y) && !ref_traj_y.empty())
    {
        current_ref_y = ref_traj_y[0];
    }

    // Check if current_ref_z is NaN and assign the first element of ref_path_z if it is
    if (std::isnan(current_ref_z) && !ref_traj_z.empty())
    {
        current_ref_z = ref_traj_z[0];
    }

    std::cout << "current_ref_x: " << current_ref_x << "\n";
    std::cout << "current_ref_y: " << current_ref_y << "\n";
    std::cout << "current_ref_z: " << current_ref_z << "\n";
}

void ref_position_cb(const geometry_msgs::Vector3::ConstPtr &msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr &msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr &msg)
{
    ref_yaw_rad = msg->data;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};

    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}

void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{

    angles = {msg->vector.x * (M_PI / 180),
              msg->vector.y * (M_PI / 180),
              msg->vector.z * (M_PI / 180)};
    angles_d = {msg->vector.x,
                msg->vector.y,
                msg->vector.z};
}

// Disturbance estimator Call back functions X, Y,Z

void dist_Fx_predInit_cb(const std_msgs::Bool::ConstPtr &msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void dist_Fy_predInit_cb(const std_msgs::Bool::ConstPtr &msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void dist_Fz_predInit_cb(const std_msgs::Bool::ConstPtr &msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void dist_data_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // Check if the incoming message has at least 3 elements
    if (msg->data.size() < 3)
    {
        ROS_ERROR("Incoming message does not have enough elements!");
        return;
    }

    // Create Float64MultiArray with 50 repeated terms for each of the first three elements
    std_msgs::Float64MultiArray dist_Fx, dist_Fy, dist_Fz;

    // Resize to  N+1 elements each
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);

    // Assign the first element of the incoming message to all 50 elements in dist_Fx
    std::fill(dist_Fx.data.begin(), dist_Fx.data.end(), msg->data[0]);

    // Assign the second element to all 50 elements in dist_Fy
    std::fill(dist_Fy.data.begin(), dist_Fy.data.end(), msg->data[1]);

    // Assign the third element to all 50 elements in dist_Fz
    std::fill(dist_Fz.data.begin(), dist_Fz.data.end(), msg->data[2]);

    // Optional: You can log the values or process them further here
    ROS_INFO("Received and processed dist_Fx, dist_Fy, dist_Fz.");

    // std::cout<<"inside callback  "<<dist_Fx.data[0]<<std::endl;
    // std::cout<<msg->data[0]<<std::endl;

    distx = msg->data[0];
    disty = msg->data[1];

    // distx=0.0;
    // disty=0.0;

    // std::cout<<"disty  "<<disty<<std::endl;
    // std::cout<<"distx  "<<distx<<std::endl;
}

void NMPC_PC::publish_wrench(struct command_struct &commandstruct)
{

    geometry_msgs::Wrench nmpc_wrench_msg;

    nmpc_wrench_msg.force.x = commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y = commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z = commandstruct.control_wrench_vec[2];

    nmpc_wrench_msg.torque.x = 0.0;
    nmpc_wrench_msg.torque.y = 0.0;
    nmpc_wrench_msg.torque.z = commandstruct.control_wrench_vec[3];

    nmpc_cmd_wrench_pub.publish(nmpc_wrench_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);
}

void NMPC_PC::publish_pred_tarjectory(struct acado_struct &traj_struct)
{
    // Create an instance of the Float32MultiArray message type
    std_msgs::Float64MultiArray pred_traj_msg;

    // Resize the data array based on the size of nmpc_pc->nmpc_struct.x
    pred_traj_msg.data.resize(NMPC_NX * (NMPC_N + 1));

    for (int i = 0; i < NMPC_NX * (NMPC_N + 1); ++i)
    {
        // pred_traj_msg.data[i] = traj_struct.x[i];
        pred_traj_msg.data[i] = nmpc_struct.x[0 + 9];
    }

    nmpc_pred_traj_pub.publish(pred_traj_msg);

    // a = nmpc_pc->nmpc_struct.x[0+9] <<endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bluerov2_nmpc_node");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);
    ros::param::get("dist_Fx_predInit_topic", dist_Fx_predInit_topic);
    ros::param::get("dist_Fy_predInit_topic", dist_Fy_predInit_topic);
    ros::param::get("dist_Fz_predInit_topic", dist_Fz_predInit_topic);
    ros::param::get("dist_Fx_data_topic", dist_Fx_data_topic);
    ros::param::get("dist_Fy_data_topic", dist_Fy_data_topic);
    ros::param::get("dist_Fz_data_topic", dist_Fz_data_topic);

    // ----------
    // Subscribers
    // ----------

     ref_traj_sub = nh.subscribe<nav_msgs::Path>("/firefly1/neptune/sampled_traj3", 1, ref_traj_cb);

    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    // vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity_body", 1, vel_cb);
    dist_Fx_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_data_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/dist", 1, dist_data_cb);
    // dist_Fx_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    // dist_Fy_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    // dist_Fz_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);
    orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

    obstacles_sub = nh.subscribe<decomp_ros_msgs::PolyhedronArray>("/firefly1/neptune/poly_safe", 1, obstacles_cb);

    neptuneTether_sub = nh.subscribe<visualization_msgs::Marker>("/firefly1/neptune/tether_rough", 1, neptuneTetherCallback);
    // neptuneTetherArray_sub = nh.subscribe<visualization_msgs::Marker>("/firefly1/neptune/tether_rough_array", 1, neptuneTetherCallback);
    // neptune_state_sub = nh.subscribe("/firefly2/state", 10, neptune_state_cb);

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wrench_pub = nh.advertise<geometry_msgs::Wrench>("/mobula/rov/wrench", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, true);

    nmpc_pred_traj_pub = nh.advertise<std_msgs::Float64MultiArray>("nmpc_predicted_trajectory", 1, true);

    s_sdot_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1, true);

    odom_point_pub = nh.advertise<geometry_msgs::PointStamped>("odom_point", 10);




    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // Roslaunch parameters
    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("yaw_control", nmpc_struct.yaw_control);
    ros::param::get("online_ref_yaw", online_ref_yaw);
    ros::param::get("use_dist_estimates", use_dist_estimates);

    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);
    int u_idx = 0;
    ros::param::get("F_x_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_y_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_z_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("Mz_ref", nmpc_struct.U_ref(u_idx++));

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_z", nmpc_struct.W(w_idx++));
    ros::param::get("W_u", nmpc_struct.W(w_idx++));
    ros::param::get("W_v", nmpc_struct.W(w_idx++));
    ros::param::get("W_w", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_r", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fx", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fy", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fz", nmpc_struct.W(w_idx++));
    ros::param::get("W_Mz", nmpc_struct.W(w_idx++));
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = sampleTime;

    NMPC_PC *nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);
    ref_traj_x.resize(20);
    ref_traj_y.resize(20);
    ref_traj_z.resize(20);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;

    angles = {0, 0, 0};

    control_stop = false;
    int loop_counter = 0; // Counter to slow down the increment of count_traj

    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (ros::ok() && !control_stop)
        {

            t_cc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_cc_loop - (int)(t_cc_loop)), (double)(sampleTime)) == 0)
                std::cout << "loop time for outer NMPC: " << t_cc_loop << " (sec)"
                          << "\n";

            // Setting up state-feedback [x,y,z,u,v,w,psi,r]

            // initialize_reference_if_Empty();

            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              angles.at(2),
                              current_vel_rate.at(5)};

            current_ref_x = ref_traj_x[count_traj];
            current_ref_y = ref_traj_y[count_traj];
            current_ref_z = ref_traj_z[count_traj];

            if (loop_counter % 10 == 0 && count_traj < size_refs - 1)
            {
                count_traj++;
            }

            ref_trajectory = {0.0, // x
                              0.0, // y
                              0.0, // z
                              0.0, // u
                              0.0, // v
                              0.0, // w
                              0.0,
                              0.0};

            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";

            // std::cout << "ref  yaw = "<< ref_yaw_rad << std::endl ;
            // std::cout << "current yaw = "<< angles.at(2) << std::endl ;

            double F_d_max = 50;

            // cap disturbance to F_dmax
            for (size_t i = 0; i < dist_Fx.data.size(); ++i)
            {
                dist_Fx.data[i] = std::min(std::max(dist_Fx.data[i], -F_d_max), F_d_max);
                dist_Fy.data[i] = std::min(std::max(dist_Fy.data[i], -F_d_max), F_d_max);
                dist_Fz.data[i] = std::min(std::max(dist_Fz.data[i], -F_d_max), F_d_max);
            }

            online_data.distFx.resize(NMPC_N + 1);
            online_data.distFy.resize(NMPC_N + 1);

            std::fill(online_data.distFx.begin(), online_data.distFx.end(), distx);
            std::fill(online_data.distFy.begin(), online_data.distFy.end(), disty);

            online_data.distFz = dist_Fx.data_zeros;

            std::cout << dist_Fx.data[0] << " (sec)" << std::endl;

            // obstacle avoidance algorithm:

            // get conact point
            geometry_msgs::Point *contactPoint = getFirstContactPoint(tether_positions);

         //  std::vector<double> rov_position = {current_pos_att.at(0), current_pos_att.at(1), 0.0};
            std::vector<double> rov_position = {neptune_x, neptune_y, 0.0};

            if (!obstacle_centers.empty())
            {
                closest_obs = findClosestObstacle(contactPoint, rov_position);
            }

            if (contactPoint == nullptr)
                CONTACT = 0;
            else
                CONTACT = 1;

            if (CONTACT == 1)
                path = CalculatePathPolynomial(rov_position, *contactPoint, closest_obs, 5);
                // Print the path points
                std::cout << "Calculated path points:" << std::endl;
                for (const auto &point : path)
                {
                    std::cout << "(" << point[0] << ", " << point[1] << ")" << std::endl;
                }
            std::cout<< "rov_position x: " << rov_position[0];
            std::cout<< "rov_position y: " << rov_position[1];

            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }

            // std::cout << "predicted dist x "<< online_data.distFx[0]<<endl;
            /// std::cout << "thrust input x1"<< nmpc_pc->nmpc_struct.x[1+9]<<endl;

            nmpc_pc->publish_pred_tarjectory(nmpc_pc->nmpc_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

            // Create a PointStamped message
            // geometry_msgs::PointStamped point_msg;
            //  point_msg.header.stamp = ros::Time::now(); // Set the timestamp
            //   point_msg.header.frame_id = "world";  // Set to your global frame here

            // point_msg.point.x = current_pos_att.at(0);
            // point_msg.point.y = current_pos_att.at(1);
            // point_msg.point.z = current_pos_att.at(2);

            // Publish the PointStamped message
            // odom_point_pub.publish(point_msg);

            ros::spinOnce();
            rate.sleep();
        }

        nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void publishOdomPoint(const std::vector<double> &current_pos_att, ros::Publisher &odom_point_pub)
{
    // Check if current_pos_att has at least 3 elements
    if (current_pos_att.size() >= 3)
    {
        // Create a PointStamped message
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now(); // Set the timestamp
        point_msg.header.frame_id = "world";       // Set to your global frame here

        // Assign values to point
        point_msg.point.x = current_pos_att.at(0);
        point_msg.point.y = current_pos_att.at(1);
        point_msg.point.z = current_pos_att.at(2);

        // Publish the PointStamped message
        odom_point_pub.publish(point_msg);
    }
    else
    {
        ROS_WARN("current_pos_att vector does not have enough elements to publish a PointStamped message.");
    }
}