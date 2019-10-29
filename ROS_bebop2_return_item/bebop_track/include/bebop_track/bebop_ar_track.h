#ifndef __BEBOP_AR_TRACK_H__
#define __BEBOP_AR_TRACK_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

//subscribe하는 topic 이름
const std::string ar_marker_pose = "/ar_pose_marker";//ar marker의 pose값을 sub 하기 위한 topic

class BebopArMarkerTrack
{
private:
    ros::NodeHandle _nodeHandle;
    
    ros::Publisher _bebopControlFromArMarkerPublisher;//ar marker pose(position 값과 orientation 값)를 기준으로 bebop을 control
    ros::Subscriber _ArMarkerPoseSubscriber;//ar marker의 pose 값을 sub
    
    geometry_msgs::Twist                                    _bebopControlMessage;//선속도,각속도를 담아 드론을 control할 변수
    ar_track_alvar_msgs::AlvarMarkers*                       AlvarMarkerMessage;//ArMarker의 pose값을 저장할 변수
    
    
    //Subscriber Call back function
    void _ArMarkerPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ARmarker);
    
public:
    explicit BebopArMarkerTrack(const ros::NodeHandle& nodeHandle);//생성자
};

#endif
