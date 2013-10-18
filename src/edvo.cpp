#include <ros/ros.h>
#include <ros/package.h>

#include <RGBDVONode.h>
#include <cvt/vision/CameraCalibration.h>
#include <cvt/vision/rgbdvo/GNOptimizer.h>
#include <cvt/vision/rgbdvo/LMOptimizer.h>

using namespace edvo;

template <class Linearizer>
void runWithLinearizerType( ros::NodeHandle& nh )
{
    typedef typename Linearizer::AlignDataType AlignDataType;
    std::string loss, optimizer;
    nh.param<std::string>( "loss", loss, "Huber" );
    nh.param<std::string>( "optimizer", optimizer, "GaussNewton" );

    if( optimizer == "LevenbergMarquardt" ){
        // LevMar
        ROS_INFO( "Optimizer -> Levenberg Marquardt" );
        if( loss == "Huber" ){
            ROS_INFO( "Loss Function -> Huber" );
            typedef cvt::LMOptimizer<AlignDataType, cvt::Huberf> OType;
            RGBDVONode<OType, Linearizer> voNode;
            ros::spin();
        } else {
            ROS_INFO( "Loss Function -> Tukey" );
            typedef cvt::LMOptimizer<AlignDataType, cvt::Tukeyf> OType;
            RGBDVONode<OType, Linearizer> voNode;
            ros::spin();
        }
    } else {
        ROS_INFO( "Optimizer -> Gauss Newton" );
        if( loss == "Huber" ){
            ROS_INFO( "Loss Function -> Huber" );
            typedef cvt::GNOptimizer<AlignDataType, cvt::Huberf> OType;
            RGBDVONode<OType, Linearizer> voNode;
            ros::spin();
        } else {
            // Tukey
            ROS_INFO( "Loss Function -> Tukey" );
            typedef cvt::GNOptimizer<AlignDataType, cvt::Tukeyf> OType;
            RGBDVONode<OType, Linearizer> voNode;
            ros::spin();
        }
    }
}

template <class WarpType>
void runWithWarpType( ros::NodeHandle& nh )
{
    typedef cvt::AlignmentData<WarpType>  AlignDataType;
    std::string linearizer;
    nh.param<std::string>( "linearizer", linearizer, "IC" );

    if( linearizer == "IC" ){
        ROS_INFO( "Linearizer -> IC" );
        runWithLinearizerType<cvt::InvCompLinearizer<AlignDataType> >( nh );
    } else if( linearizer == "FC" ){
        ROS_INFO( "Linearizer -> FC" );
        runWithLinearizerType<cvt::FwdCompLinearizer<AlignDataType> >( nh );
    } else {
        // ESM
        ROS_INFO( "Linearizer -> ESM" );
        runWithLinearizerType<cvt::ESMLinearizer<AlignDataType> >( nh );
    }
}

void run( ros::NodeHandle& nh )
{
    std::string warp;
    nh.param<std::string>( "warp", warp, "AI" );
    if( warp == "AI" ){
        ROS_INFO( "Warp -> Affine Illumination" );
        runWithWarpType<cvt::AffineLightingWarp>( nh );
    } else {
        ROS_INFO( "Warp -> Standard" );
        runWithWarpType<cvt::StandardWarp>( nh );
    }
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "edvo" );

    ros::NodeHandle nh( "~" );
    run( nh );
    return 0;
}
