#include "climb_up.h"
#include <std_msgs/Float64.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "climb_up_test");
  
  ros::NodeHandle nh;
  ros::Publisher leg_roll_pub = nh.advertise<std_msgs::Float64>("/hexapod/leg1_roll_joint_position_controller/command", 10);
  ros::Publisher leg_femur_pub = nh.advertise<std_msgs::Float64>("/hexapod/leg1_pitch1_joint_position_controller/command", 10);
  ros::Publisher leg_tibia_pub = nh.advertise<std_msgs::Float64>("/hexapod/leg1_pitch2_joint_position_controller/command", 10);
  ros::Publisher leg_tarsus_pub = nh.advertise<std_msgs::Float64>("/hexapod/leg1_pitch3_joint_position_controller/command", 10);
  std_msgs::Float64 leg_roll, leg_femur, leg_tibia, leg_tarsus;
  int leg_index = 0;
  double distance, theta1, theta2, theta3, theta4, feet_height;
  Climb_up climb_up;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate loop_rate(50);

    ROS_INFO("distance: ");
    std::cin>>distance;
    ROS_INFO("theta2: ");
    std::cin>>theta2;
    theta2 = M_PI*theta2/180.0;
    ROS_INFO("theta2: %f", theta2);
    climb_up.climb_up_calculate(&leg_index, &distance, &theta2, &theta1, &theta3, &theta4, &feet_height);
    ROS_INFO("theta1: %f", theta1);
    ROS_INFO("theta2: %f", theta2);
    ROS_INFO("theta3: %f", theta3);
    ROS_INFO("theta4: %f", theta4);
    ROS_INFO("feet_height: %f", feet_height);
    
//     leg_roll.data = theta1;
//     leg_roll_pub.publish(leg_roll);
//   
// //     leg_femur.data = theta2;
// //     leg_femur_pub.publish(leg_femur);
//   
//     leg_tibia.data = theta3;
//     leg_tibia_pub.publish(leg_tibia);
//   
//     leg_tarsus.data = theta4;
//     leg_tarsus_pub.publish(leg_tarsus);
    
    vector<Climb_up::three_order_equation> equ;
    vector<Climb_up::inter_position> inter_pos;
    Climb_up::inter_position pos;
    pos.position = 0;
    pos.speed = 0;
    inter_pos.push_back(pos);
    pos.position = theta2/2;
    pos.speed = 0.1;
    inter_pos.push_back(pos);
    pos.position = theta2;
    pos.speed = 0;
    inter_pos.push_back(pos);
    double t = 2.0;
    equ = climb_up.interpolation_equation(inter_pos, t);
    
    vector<Climb_up::three_order_equation> equ1;
    vector<Climb_up::inter_position> inter_pos1;
    Climb_up::inter_position pos1;
    pos1.position = 0;
    pos1.speed = 0;
    inter_pos1.push_back(pos1);
    pos1.position = theta1/2;
    pos.speed = 0.1;
    inter_pos1.push_back(pos1);
    pos1.position = theta1;
    pos1.speed = 0;
    inter_pos1.push_back(pos1);
    equ1 = climb_up.interpolation_equation(inter_pos1, t);
    
    vector<Climb_up::three_order_equation> equ3;
    vector<Climb_up::inter_position> inter_pos3;
    Climb_up::inter_position pos3;
    pos3.position = 0;
    pos3.speed = 0;
    inter_pos3.push_back(pos3);
    pos3.position = theta3/2;
    pos3.speed = 0.1;
    inter_pos3.push_back(pos3);
    pos3.position = theta3;
    pos3.speed = 0;
    inter_pos3.push_back(pos3);
    equ3 = climb_up.interpolation_equation(inter_pos3, t);
    
    
    vector<Climb_up::three_order_equation> equ4;
    vector<Climb_up::inter_position> inter_pos4;
    Climb_up::inter_position pos4;
    pos3.position = 0;
    pos3.speed = 0;
    inter_pos4.push_back(pos4);
    pos4.position = theta4/2;
    pos4.speed = 0.1;
    inter_pos4.push_back(pos4);
    pos4.position = theta4;
    pos4.speed = 0;
    inter_pos4.push_back(pos4);
    equ4 = climb_up.interpolation_equation(inter_pos4, t);
    

    
    
    vector<Climb_up::three_order_equation>::iterator pd;
    vector<Climb_up::three_order_equation>::iterator pd1;
    vector<Climb_up::three_order_equation>::iterator pd3;
    vector<Climb_up::three_order_equation>::iterator pd4;
    
    for(pd = equ.begin(), pd1=equ1.begin(), pd3=equ3.begin(), pd4=equ4.begin(); pd != equ.end(); pd++)
    {
      std::cout<<pd->a0<<" "<<pd->a1<<" "<<pd->a2<<" "<<pd->a3<<std::endl;
    }
    
    double cycle_length = 50.0;
    for( pd = equ.begin(); pd != equ.end(); pd++, pd1++, pd3++, pd4++ )
    {
      for(double i = 0.0; i < cycle_length; i++ )
      {
	leg_femur.data = pd->a0 + pd->a1 * (i/cycle_length*t) + pd->a2 * pow((i/cycle_length*t), 2) + pd->a3 * pow((i/cycle_length*t), 3);
	std::cout<<pd->a0<<" "<<pd->a1<<" "<<pd->a2<<" "<<pd->a3<<std::endl;
	std::cout<<leg_femur.data<<std::endl;
	leg_femur_pub.publish(leg_femur);
	
	leg_roll.data = pd1->a0 + pd1->a1 * (i/cycle_length*t) + pd1->a2 * pow((i/cycle_length*t), 2) + pd1->a3 * pow((i/cycle_length*t), 3);
	leg_roll_pub.publish(leg_roll);
	
	leg_tibia.data = pd3->a0 + pd3->a1 * (i/cycle_length*t) + pd3->a2 * pow((i/cycle_length*t), 2) + pd3->a3 * pow((i/cycle_length*t), 3);
	leg_tibia_pub.publish(leg_tibia);
	
	leg_tarsus.data = pd4->a0 + pd4->a1 * (i/cycle_length*t) + pd4->a2 * pow((i/cycle_length*t), 2) + pd4->a3 * pow((i/cycle_length*t), 3);
	leg_tarsus_pub.publish(leg_tarsus);
	
	
	loop_rate.sleep();
      }
    }
    pd = equ.end() - 1;
    leg_femur.data = pd->a0 + pd->a1 * t + pd->a2 * pow(t, 2) + pd->a3 * pow(t, 3);
    leg_femur_pub.publish(leg_femur);
    
    
  
}
