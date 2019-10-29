#include <bebop_track/bebop_ar_track.h>


void BebopArMarkerTrack::_ArMarkerPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ARmarker)
{
    AlvarMarkerMessage->markers[0].id = ARmarker->markers[0].id;
    ROS_INFO("Ar Marker id: %d", AlvarMarkerMessage->markers[0].id);
}

//생성자
BebopArMarkerTrack::BebopArMarkerTrack(const ros::NodeHandle& nodeHandle)
:_nodeHandle(nodeHandle),
_ArMarkerPoseSubscriber(_nodeHandle.subscribe(ar_marker_pose, 1, &BebopArMarkerTrack::_ArMarkerPoseCallback, this))
{

}
//_ArMarkerPoseSubscriber(_nodeHandle.subscribe(ar_marker_pose, 10, &BebopArMarkerTrack::_ArMarkerPoseCallback, this))
