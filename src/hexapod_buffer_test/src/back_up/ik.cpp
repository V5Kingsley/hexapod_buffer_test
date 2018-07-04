#include "ik.h"

static const double PI=3.141592653;

IK::IK(void)
{
  ros::param::get( "COXA_TO_CENTER_X", COXA_TO_CENTER_X ); 
  ros::param::get( "COXA_TO_CENTER_Y", COXA_TO_CENTER_Y );
  ros::param::get( "INIT_COXA_ANGLE", INIT_COXA_ANGLE );
  ros::param::get( "INIT_FOOT_POS_X", INIT_FOOT_POS_X );   //足端到髋关节的初始位置
  ros::param::get( "INIT_FOOT_POS_Y", INIT_FOOT_POS_Y );
  ros::param::get( "INIT_FOOT_POS_Z", INIT_FOOT_POS_Z );
  ros::param::get( "COXA_LENGTH", COXA_LENGTH );
  ros::param::get( "FEMUR_LENGTH", FEMUR_LENGTH );
  ros::param::get( "TIBIA_LENGTH", TIBIA_LENGTH );
  ros::param::get( "TARSUS_LENGTH", TARSUS_LENGTH );
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
}

Trig IK::getSinCos( double angle_rad )
{
  Trig body_trig;
  body_trig.sine=sin( angle_rad );
  body_trig.cosine=cos( angle_rad );
  return body_trig;
}

void IK::calculateIK(const hexapod_msgs::FeetPositions& feet, hexapod_msgs::LegsJoints* legs)
{
  double sign=1.0;
  for( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    if( leg_index<=2 ) 
    {
      sign=1.0;
    }
    else{
      sign=1.0;
    }
    
    //机体旋转角度，angular.z
    Trig A=getSinCos( feet.foot[leg_index].orientation.yaw );
    //足端到机体中心距离
    double cpr_x=INIT_FOOT_POS_X[leg_index]+COXA_TO_CENTER_X[leg_index]  + feet.foot[leg_index].position.x;
    double cpr_y = sign*( INIT_FOOT_POS_Y[leg_index] + COXA_TO_CENTER_Y[leg_index] )+ feet.foot[leg_index].position.y;
    double cpr_z =  TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index] + feet.foot[leg_index].position.z;
    //机体因旋转导致的足端位姿变化cpr*rot(z, Θ)
    double body_pos_x =  ( cpr_x * A.cosine - cpr_y * A.sine ) - cpr_x;
    double body_pos_y =  ( cpr_x * A.sine + cpr_y * A.cosine ) - cpr_y;
    double body_pos_z = cpr_z - cpr_z;
    
     // 足端到髋关节的位置x,y,z
    double feet_pos_x = INIT_FOOT_POS_X[leg_index] + feet.foot[leg_index].position.x + body_pos_x;
    double feet_pos_y =  sign *( INIT_FOOT_POS_Y[leg_index] ) + body_pos_y + feet.foot[leg_index].position.y;
    double feet_pos_z =  INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH - feet.foot[leg_index].position.z ;
    
    double femur_to_tarsus = sqrt( pow( feet_pos_x, 2 ) + pow( feet_pos_y, 2 ) ) - COXA_LENGTH;
     
    if( std::abs( femur_to_tarsus ) > ( FEMUR_LENGTH + TIBIA_LENGTH ) )
       {
            ROS_FATAL("IK Solver cannot solve a foot position that is not within leg reach!!!");
            ROS_FATAL("Shutting down so configuration can be fixed!!!");
            ros::shutdown();
            break;
        }
        
        double side_a = FEMUR_LENGTH;
        double side_a_sqr = pow( FEMUR_LENGTH, 2 );

        double side_b = TIBIA_LENGTH;
        double side_b_sqr = pow( TIBIA_LENGTH, 2 );

        double side_c = sqrt( pow( femur_to_tarsus, 2 ) + pow( feet_pos_z, 2 ) );
        double side_c_sqr = pow( side_c, 2 );

        // We are using the law of cosines on the triangle formed by the femur, tibia and tarsus joints.
	double angle_a = acos( ( side_b_sqr - side_a_sqr + side_c_sqr ) / ( 2.0 * side_b * side_c ) );
        double angle_b = acos( ( side_a_sqr - side_b_sqr + side_c_sqr ) / ( 2.0 * side_a * side_c ) );
        double angle_c = acos( ( side_a_sqr + side_b_sqr - side_c_sqr ) / ( 2.0 * side_a * side_b ) );

        // Angle of line between the femur and Tarsus joints with respect to feet_pos_z.
        double theta = atan2( femur_to_tarsus, feet_pos_z );

        // Resulting joint angles in radians.
        legs->leg[leg_index].coxa = atan2( feet_pos_y, feet_pos_x ) - INIT_COXA_ANGLE[leg_index];
        legs->leg[leg_index].femur = angle_b - ( PI/2 - theta );
        legs->leg[leg_index].tibia =  angle_c - PI/2;
        legs->leg[leg_index].tarsus =-( PI/2 - ( angle_a + ( angle_b - legs->leg[leg_index].femur ) ) );
    
  }
}



