#include <ros/ros.h>
#include <ros/package.h>

#include <RGBDVONode.h>
#include <cvt/vision/CameraCalibration.h>

using namespace cvt_ros;

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "direct_vo" );

    ros::NodeHandle nh( "~" );

    bool useCalibFile = false;
    std::string filename = "xtion_rgb.xml";

    nh.param<std::string>( "calib_file", filename, filename );
    nh.param<bool>( "use_calib_file", useCalibFile, useCalibFile );

    ROS_INFO( "Using Calibration file: %d", useCalibFile );

    RGBDVONode* voNode = 0;

    if( useCalibFile ){
        try {
            // load calibration
            cvt::String resourcePath;
            resourcePath.sprintf( "%s/resources/%s", ros::package::getPath( "cvt_ros" ).c_str(), filename.c_str() );
            ROS_INFO( "Calibration file: %s", resourcePath.c_str() );
            cvt::CameraCalibration calib;
            calib.load( resourcePath.c_str() );
            voNode = new RGBDVONode( calib.intrinsics() );
        } catch( cvt::Exception& e ){
            ROS_ERROR( "%s", e.what() );
        }

    } else {
        voNode = new RGBDVONode();
    }

    ros::spin();
    delete voNode;

    return 0;
}
