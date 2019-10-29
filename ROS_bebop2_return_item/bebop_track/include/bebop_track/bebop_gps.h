#ifndef __BEBOP_GPS_H__
#define __BEBOP_GPS_H__

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"

const bool gpsOff = false;
const bool gpsOn = true;
const bool gpsModeOff = false;
const bool gpsModeOn = true;
const bool endGoHome = false;
const bool goEnteredcoordinates = true;
const bool endGoEnteredcoordinates = false;

//파라미터 키 이름
const std::string gps_key = "/bebop/gps";
const std::string gps_mode_key = "/bebop/gpsmode";
const std::string takeoff_key = "/bebop/takeoff";
const std::string home_gps_latitude_key = "/bebop/homegps/latitude";
const std::string home_gps_longitude_key = "/bebop/homegps/longitude";
const std::string drone_gps_latitude_key = "/bebop/dronegps/latitude"; // teleop 화면에 gps latitude값을 띄우기 위한 param key
const std::string drone_gps_longitude_key = "/bebop/dronegps/longitude"; // teleop 화면에 gps longitude값을 띄우기 위한 param key
const std::string go_home_key = "/bebop/gohome";
const std::string go_entered_coordinates_key = "/bebop/goenteredcoordinates";
const std::string go_latitude_key = "/bebop/go/latitude";
const std::string go_longitude_key = "/bebop/go/longitude";

//subscribe하는 topic 이름
const std::string drone_gps = "/bebop/states/ardrone3/PilotingState/PositionChanged";//드론의 gps값을 얻기위한 topic
const std::string bebop_attitude = "/bebop/states/ardrone3/PilotingState/AttitudeChanged";//드론의 yaw값을 얻기위한 topic

class BebopGlobalPositioningSystem
{
private:
    ros::NodeHandle _nodeHandle;
    
    ros::Publisher _bebopControlFromGPSPublisher;//목표 위치와 현재 위치를 비교해서 x방향 선속도를 pub
    ros::Publisher _bebopGoHomePublisher;//home 방향으로 기체 머리를 돌리고 고도를 맞추는 pub
    ros::Subscriber _currentGPSSubscriber;//현재 drone이 위치한 gps값을 sub
    ros::Subscriber _currentAttitudeSubscriber;//현재 drone의 yaw값을 sub
    
    sensor_msgs::NavSatFix                                  home_position;//home으로 지정한 위치의 위도(latitude), 경도(longitude)값을 담을 변수
    bebop_msgs::Ardrone3PilotingStatePositionChanged        current_position;//드론의 현재 위도(latitude), 경도(longitude)값을 담을 변수
    bebop_msgs::Ardrone3PilotingStateAttitudeChanged        bebopAttitude;//드론의 yaw값을 담을 변수
    geometry_msgs::Twist                                    _bebopControlMessage;//선속도,각속도를 담아 드론을 control할 변수
    std_msgs::Bool                                          startNavigateHome;//true이면 home방향으로 드론 머리를 돌리고 고도를 일정 고도(bebop 내부에 정해져 있는듯하다.)로 맞춘다.
    
    bool _isTakeOff;                //is takeoff?                 yes:true, no:false
    bool _isGPS;                    //is gps?                     yes:true, no:false
    bool _isGoHome;                 //is go home?                 yes:true, no:false 
    bool _isGoEnteredCoordinates;   //is go entered coordinates?  yes:true, no:false
    
    //Subscriber Call back function
    void _currentGPSCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& currentGPS);//현재 gps값을 call back
    void _currentAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged& currentAttitude);//현재 drone의 yaw값을 call back 
    
    void getParam();//파라미터들의 상태를 얻어옴
    void returnHome();//집으로 설정된 위도(latitude),경도(longitude)로 가는 함수
    void goEnteredCoordinates();//입력한 위도,경도 값으로 가는 함수
public:
    
    void Action();//파라미터 값에 따라 drone의 행동을 결정하는 함수
    explicit BebopGlobalPositioningSystem(const ros::NodeHandle& nodeHandle);//생성자
};

#endif
