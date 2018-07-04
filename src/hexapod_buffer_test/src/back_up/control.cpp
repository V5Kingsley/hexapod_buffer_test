#include "control.h"

Control::Control()
{
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  ros::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS );
  ros::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
   ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
   ros::param::get( "STICK_FORCE", STICK_FORCE );
   setup_ = 1;
   
   
    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/cmd_vel", 1, &Control::cmd_velCallback, this );
    
    //发布的话题
    boost::format roll;
    boost::format pitch1;
    boost::format pitch2;
    boost::format pitch3;
    for ( int leg_index = 0,  j =1;  leg_index < NUMBER_OF_LEGS;  leg_index ++, j ++ )
    {
       roll = boost::format("/hexapod/leg%d_roll_joint_position_controller/command") % j;
       pitch1 = boost::format("/hexapod/leg%d_pitch1_joint_position_controller/command") % j;
       pitch2 = boost::format("/hexapod/leg%d_pitch2_joint_position_controller/command") % j; 
       pitch3 = boost::format("/hexapod/leg%d_pitch3_joint_position_controller/command") % j;
      leg_topic[leg_index] = roll.str();
      leg_topic[leg_index+1] = pitch1.str();
      leg_topic[leg_index+2] = pitch2.str();
      leg_topic[leg_index+3] = pitch3.str();
      leg_roll_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index], 10 );
      leg_pitch1_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+1], 10 );
      leg_pitch2_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+2], 10 );
      leg_pitch3_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+3], 10 );
    }

 //开总线
  bushandle = smOpenBus("/dev/ttyUSB0");
  if (bushandle < 0)
  {
    ROS_INFO("open bus error");
  }
  else
  {
    ROS_INFO("open bus succeed");
  }
  
  //使能电机
  for(int i=1; i<5; i++)
  {
    smSetParameter(bushandle, i, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
    smSetParameter(bushandle, i, SMP_CONTROL_BITS2, SMP_CB2_ENABLE);
  }
  
  //初始化总线
  for(int i=0; i<4; i++)
  {
    smBufferedInit(&axis[i], bushandle, i+1, 600, SMP_ACTUAL_POSITION_FB, SM_RETURN_VALUE_16B);
//     smSetParameter(bushandle,i+1,SMP_DRIVE_FLAGS,axis[i].driveFlagsBeforeInit&(0xfffff7ff));
  }
 
 /*   //吸盘
  * 
    boost::format stick;
    for ( int leg_index = 0,  j=1; leg_index < NUMBER_OF_LEGS; leg_index++, j++ )
    {
      stick = boost::format( "leg%d_stick3" ) % j;
      force_to_stick[leg_index] = stick.str();
    }
    
    feet_position = nh_.advertise<hexapod_msgs::FeetPositions>("feet_position", 10); */
 
  
}

void Control::robotInit()
{
   //关节转动角度
  for ( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    leg_roll[leg_index].data = 0.0;
    leg_pitch1[leg_index].data = 0.0;
    leg_pitch2[leg_index].data = 0.0;
    leg_pitch3[leg_index].data = 0.0;
  }
       for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++  )
       {
	 leg_roll_p[leg_index].publish( leg_roll[leg_index] );
	 leg_pitch1_p[leg_index].publish( leg_pitch1[leg_index] );
	 leg_pitch2_p[leg_index].publish( leg_pitch2[leg_index] );
	 leg_pitch3_p[leg_index].publish( leg_pitch3[leg_index] );
      }
  
}

 

void Control::publishJointStates( const hexapod_msgs::LegsJoints &legs, int &cycle_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet )
{
      for ( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    leg_roll[leg_index].data = legs.leg[leg_index].coxa;
    leg_pitch1[leg_index].data = legs.leg[leg_index].femur;
    leg_pitch2[leg_index].data = legs.leg[leg_index].tibia;
    leg_pitch3[leg_index].data = legs.leg[leg_index].tarsus;
  }
  
  //发布关节角度话题
       for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++  )
       {
	 leg_roll_p[leg_index].publish( leg_roll[leg_index] );
	 leg_pitch1_p[leg_index].publish( leg_pitch1[leg_index] );
	 leg_pitch2_p[leg_index].publish( leg_pitch2[leg_index] );
	 leg_pitch3_p[leg_index].publish( leg_pitch3[leg_index] );
      }
      
    //  feet_position.publish(*feet);

/*   //吸盘吸附力控制
//   if ( cycle_period_ == 1 )
//   {
// 
//      for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
//   {
// 
//      if (  cycle_leg_number_[leg_index] == 0)
//      {
//       srv.request.body_name = force_to_stick[leg_index];
//       srv.request.reference_frame = force_to_stick[leg_index];
//       srv.request.wrench.force.z = 0;
//       srv.request.duration.sec = 2;
//       force_client.call(srv);
//      }  
//      if (  cycle_leg_number_[leg_index] == 1 )
//     {
//       srv.request.body_name = force_to_stick[leg_index];
//       srv.request.reference_frame = force_to_stick[leg_index];
//       srv.request.wrench.force.z = - STICK_FORCE;
//       srv.request.duration.sec = 3;
//       force_client.call(srv);
//     }
//   }
//     ros::Duration(15).sleep(); 
   }*/
 
}

//订阅发布的速度信息
void Control::cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg )
{
  if (cmd_vel_msg->linear.x>0.1||cmd_vel_msg->linear.x<-0.1)
  {
    ROS_FATAL("The linear.x exceeds the upper limit, set it to max: 0.1.");
    cmd_vel_incoming_.linear.x = (cmd_vel_msg->linear.x > 0 ? 0.1 : -0.1);
  }
  else
  {
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
  }
  
    if (cmd_vel_msg->linear.y>0.1|| cmd_vel_msg->linear.y<-0.1)
  {
    ROS_FATAL("The linear.y exceeds the upper limit, set it to max: 0.1.");
    cmd_vel_incoming_.linear.y = (cmd_vel_msg->linear.y > 0 ? 0.1 : -0.1);;
  }
  else
  {
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
  }
  
  if (cmd_vel_msg->angular.z>0.2 || cmd_vel_msg->angular.z<-0.2)
  {
    ROS_FATAL("The angular.z exceeds the upper limit, set it to max: 0.2.");
    cmd_vel_incoming_.angular.z = (cmd_vel_msg->angular.z > 0 ? 0.2 : -0.2);;
  }
  else
  {
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
  }
}

//速度转化成歩幅
void Control::partitionCmd_vel( geometry_msgs::Twist *cmd_vel )
{
    // Instead of getting delta time we are calculating with a static division
    double dt = VELOCITY_DIVISION;

    double delta_th = cmd_vel_incoming_.angular.z * dt;
    double delta_x = ( cmd_vel_incoming_.linear.x * cos( delta_th ) - cmd_vel_incoming_.linear.y * sin( delta_th ) ) * dt;
    double delta_y = ( cmd_vel_incoming_.linear.x * sin( delta_th ) + cmd_vel_incoming_.linear.y * cos( delta_th ) ) * dt;
    cmd_vel->linear.x = delta_x;
    cmd_vel->linear.y = delta_y;
    cmd_vel->angular.z = delta_th;
}

void Control::feedDrives()
{
  smint32 positions[4][64];
  smint32 readData[4][64];
  smint32 readDataAmount;
  smint32 freeSpace;
  int i, j;
  
  int minimumBufferFreeBytes = axis[0].bufferLength - 0.9 * axis[0].bufferLength;
  
  smBufferedGetFree(&axis[0], &freeSpace);
  
  if(axis[0].bufferFill==0)
  {
    ROS_INFO("Empty buffer detected (the first fill, or buffer underrun)");
  }
  
  while(smBufferedGetMaxFillSize(&axis[0], freeSpace)>1 && freeSpace >= minimumBufferFreeBytes)
  {
    int maxpoints = smBufferedGetMaxFillSize(&axis[0], freeSpace);
    ROS_INFO("maxpoints: %d", maxpoints);
 //从gait提取maxpoints个点
    for(j=0; j<maxpoints; j++)
    {
      partitionCmd_vel( &cmd_vel_ );
      gait.gaitCycle( cmd_vel_, &feet_);
      ik.calculateIK( feet_, &legs_ );
      publishJointStates( legs_, gait.cycle_period_, gait.cycle_leg_number_, &feet_);
      
      positions[0][j] = round(4096*(3005640.0/1300.0)*(legs_.leg[0].coxa/M_PI*360)/360.0);
      ROS_INFO("position[0][%d]: %d", j, positions[0][j]);
      positions[1][j] = round(4096*(3005640.0/1300.0)*(-legs_.leg[0].femur/M_PI*360)/360.0);
      ROS_INFO("position[1][%d]: %d", j, positions[1][j]);
      positions[2][j] = round(4096*(3005640.0/1300.0)*(-legs_.leg[0].tibia/M_PI*360)/360.0);
      ROS_INFO("position[2][%d]: %d", j, positions[2][j]);
      positions[3][j] = round(4096*(3005640.0/1300.0)*(-legs_.leg[0].tarsus/M_PI*360)/360.0);
      ROS_INFO("position[3][%d]: %d", j, positions[3][j]);
    }
    
//send to devies, and receive read data back
  smint32 bytesFilled;
  for(i=0; i<4; i++)
  {
 /*   smint32 Temp;
    smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_TRAJ_PLANNER_VEL);
    smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, 10000);
    smExecuteCommandQueue(bushandle, i+1);
    smGetQueuedSMCommandReturnValue(bushandle, &Temp);
    smGetQueuedSMCommandReturnValue(bushandle, &Temp);*/
    smBufferedFillAndReceive(&axis[i], maxpoints, positions[i], &readDataAmount, readData[i], &bytesFilled);
  }
  freeSpace -= bytesFilled;
  }
  
  smBufferedRunAndSyncClocks(&axis[0]);
  
  
}




