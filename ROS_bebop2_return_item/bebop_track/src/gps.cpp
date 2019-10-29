#include <bebop_track/bebop_gps.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_gps_node");
    ros::NodeHandle nodeHandle;
    
    BebopGlobalPositioningSystem bebopGPS(nodeHandle);
    
    bebopGPS.Action();

    return 0;
}
