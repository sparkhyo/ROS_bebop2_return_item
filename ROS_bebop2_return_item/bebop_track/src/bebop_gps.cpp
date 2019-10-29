#include <bebop_track/bebop_gps.h>
#include <math.h>

#define ANGULAR_SPEED 0.3
#define LINEAR_SPEED 0.3
#define DOWN_ANGULAR_SPEED 0.01
#define DOWN_ANG_SPEED_GAP 0.3
#define STOP_DISTANCE 3.0

bool ready_to_get_homeGPS = true;
bool navigate_home = true;
bool basic_angle_set = true;
bool target_angle_set = true;
int num = 0;

/*
    bebop 2
    
    Max horizontal speed: 16 m/s
    Max upward speed: 6 m/s
    Max rotation speed: 200 degree/s
    Max tilt speed: 300 degree/s
    Max tilt: 35 degree
*/


/////////////////////////////////////////////
double degree_to_radian(double degree){
	return degree * (M_PI / 180);
}

double radian_to_degree(double radian){
	return radian * (180 / M_PI);
}
/////////////////////////////////////////////
//현재 드론이 위치한 위도,경도와 드론이 가야하는 위도,경도를 가지고 거리를 계산하는 함수 
double calDistance(double lat1, double lon1, double lat2, double lon2){

    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(degree_to_radian(lat1)) * sin(degree_to_radian(lat2)) + cos(degree_to_radian(lat1))
          * cos(degree_to_radian(lat2)) * cos(degree_to_radian(theta));
    dist = acos(dist);
    dist = radian_to_degree(dist);

    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;    // 단위 mile 에서 km 변환.
    dist = dist * 1000.0;      // 단위  km 에서 m 로 변환

    return dist;
}
////////////////////////////////////////////////
//현재 드론이 위치한 위도,경도가 드론이 가야하는 위도 경도를 기준으로 몇도 꺾여있는지 구하는 함수.
//왼쪽으로 0.0 ~ 3.141592 rad 값이 나오고 오른쪽으로 0.0 ~ 3.141592 rad의 값이 나온다. bebop의 yaw값과 다르게 왼쪽 오른쪽 둘 다 양수가 나온다.
double bearingP1toP2(double P1_latitude, double P1_longitude, double P2_latitude, double P2_longitude)
{
    // 현재 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
    double Cur_Lat_radian = P1_latitude * (3.141592 / 180);
    double Cur_Lon_radian = P1_longitude * (3.141592 / 180);


    // 목표 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
    double Dest_Lat_radian = P2_latitude * (3.141592 / 180);
    double Dest_Lon_radian = P2_longitude * (3.141592 / 180);

    // radian distance
    double radian_distance = 0;
    radian_distance = acos(sin(Cur_Lat_radian) * sin(Dest_Lat_radian) + cos(Cur_Lat_radian) * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian));

    // 목적지 이동 방향을 구한다.(현재 좌표에서 다음 좌표로 이동하기 위해서는 방향을 설정해야 한다. 라디안값이다.
    double radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian) * cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)));        // acos의 인수로 주어지는 x는 360분법의 각도가 아닌 radian(호도)값이다.        
    
    /*double true_bearing = 0;
    if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0)
    {
        true_bearing = radian_bearing * (180 / 3.141592);
        true_bearing = 360 - true_bearing;
    }
    else
    {
        true_bearing = radian_bearing * (180 / 3.141592);
    }*/

    //return true_bearing; //unit: degree
    return radian_bearing; //unit: radian
}
////////////////////////////////////////////
void BebopGlobalPositioningSystem::_currentGPSCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& currentGPS)
{   
    ros::Rate loop_rate(50);
    
    _nodeHandle.getParam(takeoff_key, _isTakeOff);
    _nodeHandle.getParam(gps_key, _isGPS);
    
    if(_isTakeOff && _isGPS)
    {
        if(ready_to_get_homeGPS) // 처음 이륙한 위치를 home위치로 설정
        {
            home_position.latitude = currentGPS.latitude;
            home_position.longitude = currentGPS.longitude;
            _nodeHandle.setParam(home_gps_latitude_key, home_position.latitude); //사용자 인터페이스 화면에 표시해주기 위해서
            _nodeHandle.setParam(home_gps_longitude_key, home_position.longitude);//사용자 인터페이스 화면에 표시해주기 위해서
            ready_to_get_homeGPS = false; //home_position 값을 다시 받지 않기 위해 ready_to_get_homeGPS를 false로 변경
        }
        
        current_position.latitude  = currentGPS.latitude; // 위도
        current_position.longitude = currentGPS.longitude; // 경도   
        
        _nodeHandle.setParam(drone_gps_latitude_key, current_position.latitude);
        _nodeHandle.setParam(drone_gps_longitude_key, current_position.longitude);
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
}


void BebopGlobalPositioningSystem::returnHome()
{
    ros::Rate loop_rate(50);
    
    while(_isGoHome) // h/H 키가 눌렸으면
    {       
        startNavigateHome.data = true;
        _bebopGoHomePublisher.publish(startNavigateHome); // /bebop/autoflight/navigate_home 토픽에 true를 pub, home gps좌표로 머리를 틀고 고도를 맞춘다.
            
        //현재 gps와 목표 gps 사이의 거리가 STOP_DISTANCE보다 크면 bebop2의 linear.x에 LINEAR_SPEED를 넣어주고 pub해준다. 
        if(calDistance(home_position.latitude, home_position.longitude, current_position.latitude, current_position.longitude) > STOP_DISTANCE)
        {          
            _bebopControlMessage.linear.x = LINEAR_SPEED;
            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
            ros::spinOnce();//spinOnce를 하면 자신을 포함한 모든 콜백함수가 호출된다. 
            loop_rate.sleep();
            continue;//
        }
        
        _bebopControlMessage.linear.x = 0.0;
        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
        navigate_home = true;
        _nodeHandle.setParam(go_home_key, endGoHome);
        break;
        //_nodeHandle.setParam(gps_key, gpsOff);    
    }
}

void BebopGlobalPositioningSystem::goEnteredCoordinates()
{
    ros::Rate loop_rate(50);
      
    double test_target_angle = bearingP1toP2(current_position.latitude, current_position.longitude, home_position.latitude, home_position.longitude);
    double test_distance = calDistance(home_position.latitude, home_position.longitude, current_position.latitude, current_position.longitude);
    
    //std::cout << "Target angle:" << test_target_angle << std::endl;
    //std::cout << "Bebop  angle:" << bebopAttitude.yaw << std::endl;
    //std::cout << "Test distance:" << test_distance << std::endl;
    //std::cout << "_isGoEnteredCoordinates state:" << _isGoEnteredCoordinates << std::endl;
    
    while(_isGoEnteredCoordinates)
    {
        //std::cout << "Entered go coordinates..." << std::endl;
        double go_latitude = 0.0;
        double go_longitude = 0.0;
        double target_angle = 0.0;
        //const bebop_msgs::Ardrone3PilotingStateAttitudeChanged currentAttitude;
        _nodeHandle.getParam(go_latitude_key, go_latitude);
        _nodeHandle.getParam(go_longitude_key, go_longitude);
        target_angle = bearingP1toP2(current_position.latitude, current_position.longitude, go_latitude, go_longitude);
        //ROS_INFO("target_angle : %lf", target_angle);
        
        
        ///////
        if(target_angle_set)
        {
            //std::cout << "--- target angle set ---" << std::endl;
            //사분면에 따라 다르게 회전
            if((go_latitude < current_position.latitude) && (go_longitude < current_position.longitude)) // 목표위치 기준 1사분면이면
            {
                if(0.0 < bebopAttitude.yaw && bebopAttitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    
                    if(target_angle - fabs(bebopAttitude.yaw) < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < fabs(bebopAttitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = (target_angle - fabs(bebopAttitude.yaw))/1.5; // 왼쪽으로 회전
                    _bebopControlMessage.angular.z = ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();                  
                }
                else if(-(1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -0.0) // yaw state 2
                {   
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle - fabs(bebopAttitude.yaw) < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < fabs(bebopAttitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = (target_angle - fabs(bebopAttitude.yaw))/1.5; // 왼쪽으로 회전
                    _bebopControlMessage.angular.z = ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(fabs(bebopAttitude.yaw) < target_angle)
                    {
                        if(target_angle - fabs(bebopAttitude.yaw) < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle < fabs(bebopAttitude.yaw))
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //_bebopControlMessage.angular.z = (target_angle - fabs(bebopAttitude.yaw))/1.5; // 왼쪽으로 회전
                        _bebopControlMessage.angular.z = ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(fabs(bebopAttitude.yaw) >= target_angle)
                    {
                        if(fabs(bebopAttitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(fabs(bebopAttitude.yaw) < target_angle) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = -(fabs(bebopAttitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                        _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if((1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle + bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle > bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = -(target_angle + bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                continue;
            }//Quadrant 1 end
            else if((go_latitude < current_position.latitude) && (go_longitude > current_position.longitude))//목표위치 기준 2사분면이면
            {
                if(0.0 < bebopAttitude.yaw && bebopAttitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    
                    if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < bebopAttitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                    
                }
                else if(-(1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -0.0) // yaw state 2
                {   
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < bebopAttitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    
                    if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                    
                }
                else if((1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(bebopAttitude.yaw > target_angle)
                    {
                        if(bebopAttitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(bebopAttitude.yaw < target_angle) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = (bebopAttitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                        _bebopControlMessage.angular.z = ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                
                    else if(bebopAttitude.yaw <= target_angle)
                    {
                        if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle < bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                        _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }                
                continue;               
            }//Quadrant 2 end
            else if((go_latitude > current_position.latitude) && (go_longitude > current_position.longitude))//목표위치 기준 3사분면이면
            {
                if(0.0 < bebopAttitude.yaw && bebopAttitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    
                    if(bebopAttitude.yaw < target_angle)
                    {
                        if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle < bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                        _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(bebopAttitude.yaw >= target_angle)
                    {  
                        if(bebopAttitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle > bebopAttitude.yaw)
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //_bebopControlMessage.angular.z = (bebopAttitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                        _bebopControlMessage.angular.z = ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if(-(1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -0.0) // yaw state 2
                {   
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle - bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < bebopAttitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = -(target_angle - bebopAttitude.yaw)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if((1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(bebopAttitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle > bebopAttitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = (bebopAttitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                    _bebopControlMessage.angular.z = ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                continue;
            }//Quadrant 3 end
            else if((go_latitude > current_position.latitude) && (go_longitude < current_position.longitude))//목표위치 기준 4사분면이면
            {
                if(0.0 < bebopAttitude.yaw && bebopAttitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle + bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < fabs(bebopAttitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = (target_angle + bebopAttitude.yaw)/1.5; // 왼쪽으로 회전
                    _bebopControlMessage.angular.z = ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                    
                }
                else if(-(1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -0.0) // yaw state 2
                {   
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(fabs(bebopAttitude.yaw) < target_angle)
                    {
                        if(target_angle + bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle < fabs(bebopAttitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = (target_angle + bebopAttitude.yaw)/1.5; // 왼쪽으로 회전
                        _bebopControlMessage.angular.z = ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(fabs(bebopAttitude.yaw) >= target_angle)
                    {  
                        if(fabs(bebopAttitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            ros::spinOnce();
                            loop_rate.sleep();
                            
                            if(target_angle > fabs(bebopAttitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                _bebopControlMessage.angular.z = 0.0;
                                _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                                
                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;    
                        }
                        //_bebopControlMessage.angular.z = -(fabs(bebopAttitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                        _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if(-M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(fabs(bebopAttitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle > fabs(bebopAttitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;    
                    }
                    //_bebopControlMessage.angular.z = -(fabs(bebopAttitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                    _bebopControlMessage.angular.z = -ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if((1.0/2.0)*M_PI < bebopAttitude.yaw && bebopAttitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl; 
                    //std::cout << "bebopAttitude.yaw:" << bebopAttitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;
                    
                    if(target_angle + bebopAttitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        _bebopControlMessage.angular.z = DOWN_ANGULAR_SPEED;
                        _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                        ros::spinOnce();
                        loop_rate.sleep();
                        
                        if(target_angle < fabs(bebopAttitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            _bebopControlMessage.angular.z = 0.0;
                            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                            
                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //_bebopControlMessage.angular.z = (target_angle + bebopAttitude.yaw)/1.5; // 왼쪽으로 회전
                    _bebopControlMessage.angular.z = ANGULAR_SPEED;
                    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
                    ros::spinOnce();
                    loop_rate.sleep();
                } 
                continue;
            }//Quadrant 4 end
        }//target angle set end
    
        
        //회전 후 직진
        if(calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude) > STOP_DISTANCE)
        {
            //_bebopControlMessage.linear.x = calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude)/20.0;   
            _bebopControlMessage.linear.x = LINEAR_SPEED;
            _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
            
            //std::cout << "bebop speed:" << _bebopControlMessage.linear.x << std::endl;
            //std::cout << "distance:" << _bebopControlMessage.linear.x*10.0 << std::endl;
                   
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        
        //std::cout << "STOP!" << std::endl;
        _nodeHandle.setParam(go_entered_coordinates_key, endGoEnteredcoordinates);
        //std::cout << "END move" << std::endl;
        break;
    }// while end
     
    _bebopControlMessage.linear.x = 0.0;
    _bebopControlMessage.angular.z = 0.0;
    _bebopControlFromGPSPublisher.publish(_bebopControlMessage);
    _nodeHandle.setParam(go_entered_coordinates_key, endGoEnteredcoordinates);
    _nodeHandle.getParam(go_entered_coordinates_key, _isGoEnteredCoordinates);
    //std::cout << "go entered coordinates State:" << _isGoEnteredCoordinates << std::endl;
    //std::cout << "Not Go Entered Coordinates State!!! DON'T MOVE!!!" << std::endl;
    basic_angle_set = true;
    target_angle_set = true;
    

}//_currentGoEnteredCoordinates callback end



void BebopGlobalPositioningSystem::_currentAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged& currentAttitude)
{
    bebopAttitude.yaw = currentAttitude.yaw;//드론이 북쪽을 바라볼 때 yaw값이 0이 나온다. 드론이 북쪽을 바라볼 때 왼쪽으로 -0.0 ~ -3.141592 rad, 오른쪽으로 0.0 ~ 3.141592의 값을 가진다.   
}


void BebopGlobalPositioningSystem::getParam()
{   
    _nodeHandle.getParam(takeoff_key, _isTakeOff);//드론이 takeoff 했는지 _isTakeOff에 상태를 받아옴
    _nodeHandle.getParam(gps_key, _isGPS);//드론의 gps를 사용할 준비가 됐는지 isGPS에 상태를 받아옴
    _nodeHandle.getParam(go_home_key, _isGoHome); //gps 기능에서 go home키 (h/H)키가 눌렸는지 _isGoHome에 상태를 받아옴
    _nodeHandle.getParam(go_entered_coordinates_key, _isGoEnteredCoordinates);//gps 기능에서 go entered coordinates키가 눌리고 좌표가 입력됬는지 _isGoEnteredCoordinates에 상태를 받아옴
       
}

void BebopGlobalPositioningSystem::Action()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        getParam();
        
        if(_isGoHome)
        {
            returnHome();
        }
        else if(_isGoEnteredCoordinates)
        {
            goEnteredCoordinates();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//생성자
BebopGlobalPositioningSystem::BebopGlobalPositioningSystem(const ros::NodeHandle& nodeHandle)
:_nodeHandle(nodeHandle),
_bebopControlFromGPSPublisher(_nodeHandle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10)),
_bebopGoHomePublisher(_nodeHandle.advertise<std_msgs::Bool>("/bebop/autoflight/navigate_home", 10)),
_currentGPSSubscriber(_nodeHandle.subscribe(drone_gps, 1, &BebopGlobalPositioningSystem::_currentGPSCallback, this)),
_currentAttitudeSubscriber(_nodeHandle.subscribe(bebop_attitude, 1, &BebopGlobalPositioningSystem::_currentAttitudeCallback, this)),
_isTakeOff(false),
_isGPS(false),
_isGoHome(false),
_isGoEnteredCoordinates(false)
{
    
}
