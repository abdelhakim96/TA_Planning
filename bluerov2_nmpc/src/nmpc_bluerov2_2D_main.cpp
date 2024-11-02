/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   Jan 2023
 *
 */

#include <nmpc_bluerov2_2D_main.h>

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace ros;
double sampleTime = 0.02;

double distx;
double disty;
double distz;
double current_ref_x ;
double current_ref_y ;
double current_ref_z;

double test_x;
double test_y;

std::vector<double> ref_traj_x;
std::vector<double> ref_traj_y;
std::vector<double> ref_traj_z;
int size_refs=10;
int count_traj =0;
//Neptune:
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


void initialize_reference_if_Empty() {
    // Check if current_ref_x is NaN and assign the first element of ref_path_x if it is
    if (std::isnan(current_ref_x) && !ref_traj_x.empty()) {
        current_ref_x = ref_traj_x[0];
    }
    
    // Check if current_ref_y is NaN and assign the first element of ref_path_y if it is
    if (std::isnan(current_ref_y) && !ref_traj_y.empty()) {
        current_ref_y = ref_traj_y[0];
    }
    
    // Check if current_ref_z is NaN and assign the first element of ref_path_z if it is
    if (std::isnan(current_ref_z) && !ref_traj_z.empty()) {
        current_ref_z = ref_traj_z[0];
    }
    
    std::cout << "current_ref_x: " << current_ref_x << "\n";
    std::cout << "current_ref_y: " << current_ref_y << "\n";
    std::cout << "current_ref_z: " << current_ref_z << "\n";
}





void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_yaw_rad = msg->data;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
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



void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}


void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{   
    
    angles = {msg->vector.x*(M_PI/180),
                   msg->vector.y*(M_PI/180),
                   msg->vector.z*(M_PI/180)};   
    angles_d ={msg->vector.x,
                   msg->vector.y,
                   msg->vector.z}; 
}


// Disturbance estimator Call back functions X, Y,Z

void dist_Fx_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void dist_Fy_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void dist_Fz_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}


void dist_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Check if the incoming message has at least 3 elements
    if (msg->data.size() < 3) {
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

    distx=msg->data[0];
    disty=msg->data[1];

    //distx=0.0;
    //disty=0.0;

   // std::cout<<"disty  "<<disty<<std::endl;
   // std::cout<<"distx  "<<distx<<std::endl;

}




void NMPC_PC::publish_wrench(struct command_struct& commandstruct)
{

    geometry_msgs::Wrench nmpc_wrench_msg;

    
    
    nmpc_wrench_msg.force.x =    commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y =    commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z =    commandstruct.control_wrench_vec[2];

    nmpc_wrench_msg.torque.x =    0.0;
    nmpc_wrench_msg.torque.y =    0.0;
    nmpc_wrench_msg.torque.z =   commandstruct.control_wrench_vec[3];

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


void NMPC_PC::publish_pred_tarjectory(struct acado_struct& traj_struct)
{
      // Create an instance of the Float32MultiArray message type
    std_msgs::Float64MultiArray pred_traj_msg;

    // Resize the data array based on the size of nmpc_pc->nmpc_struct.x
   pred_traj_msg.data.resize(NMPC_NX * (NMPC_N + 1));


       for (int i = 0; i < NMPC_NX * (NMPC_N + 1); ++i)
    {
       // pred_traj_msg.data[i] = traj_struct.x[i];
        pred_traj_msg.data[i] =  nmpc_struct.x[0+9];
    }
   

  nmpc_pred_traj_pub.publish(pred_traj_msg);
  
    // a = nmpc_pc->nmpc_struct.x[0+9] <<endl;
 
}


int main(int argc, char** argv)
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
    //vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity_body", 1, vel_cb);
    dist_Fx_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_data_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/dist", 1, dist_data_cb);
   // dist_Fx_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    //dist_Fy_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    //dist_Fz_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);
    orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

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

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
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

     angles = { 0,0,0};
    
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
            
           initialize_reference_if_Empty();
            


            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              angles.at(2),
                              current_vel_rate.at(5)
                              };

             
            cout<<"!!!!!!!!!!ref_traj_x[count_traj]"<<test_x <<endl;
            current_ref_x = ref_traj_x[count_traj];
            current_ref_y = ref_traj_y[count_traj];
            current_ref_z = ref_traj_z[count_traj];
             

            if (loop_counter % 10 == 0 && count_traj < size_refs - 1)
            {
                count_traj++;
            }

           
                    ref_trajectory = {test_x,  //x
                                     test_y,  //y
                                     0.0,   //z
                                      0.0,   //u
                                      0.0,   //v
                                      0.0,   //w
                                      0.0,
                                      0.0
                             };                   



            
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

 

          //cap disturbance to F_dmax
                for (size_t i = 0; i < dist_Fx.data.size(); ++i) {
                    dist_Fx.data[i] = std::min(std::max(dist_Fx.data[i], -F_d_max), F_d_max);
                    dist_Fy.data[i] = std::min(std::max(dist_Fy.data[i], -F_d_max), F_d_max);
                    dist_Fz.data[i] = std::min(std::max(dist_Fz.data[i], -F_d_max), F_d_max);
                }



     online_data.distFx.resize(NMPC_N + 1);
     online_data.distFy.resize(NMPC_N + 1);
                
         
             std::fill(online_data.distFx.begin(), online_data.distFx.end(), distx);
             std::fill(online_data.distFy.begin(), online_data.distFy.end(), disty);


           online_data.distFz = dist_Fx.data_zeros;
           //std::cout<<online_data.distFx[0]<<std::endl;
          // std::cout<<"online_data.distFx[29]:  "<<online_data.distFx[29]<<std::endl;


          //std::cout << "\033[1;31m" << "online_data = " << online_data.distFx[0] << " (sec)" << "\033[0m" << std::endl;
          //std::cout << "\033[1;31m" << "online_data = " << online_data.distFy[0] << " (sec)" << "\033[0m" << std::endl;
          //std::cout << "\033[1;31m" << "online_data = " << online_data.distFz[0] << " (sec)" << "\033[0m" << std::endl;

          std::cout <<  dist_Fx.data[0] << " (sec)"<< std::endl;

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

            //std::cout << "predicted dist x "<< online_data.distFx[0]<<endl;
           /// std::cout << "thrust input x1"<< nmpc_pc->nmpc_struct.x[1+9]<<endl;

            nmpc_pc->publish_pred_tarjectory(nmpc_pc->nmpc_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            
            
                    // Create a PointStamped message
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now(); // Set the timestamp
         point_msg.header.frame_id = "world";  // Set to your global frame here
        point_msg.point.x = current_pos_att.at(0);
        point_msg.point.y = current_pos_att.at(1);
        point_msg.point.z = current_pos_att.at(2);

        // Publish the PointStamped message
        odom_point_pub.publish(point_msg);
            
            
            
            
            ros::spinOnce();
            rate.sleep();
            
        }
        



        
        nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);
        



        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
