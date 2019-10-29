#include <bebop_track/bebop_ar_track.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_ar_track_node");
    ros::NodeHandle nodeHandle;
    
    ros::Rate loop_rate(50);
    
    BebopArMarkerTrack bebopAr(nodeHandle);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
