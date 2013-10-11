#include "RGBDVONode.h"

namespace cvt_ros {

    RGBDVONode::RGBDVONode() :
        RGBDSubscriber(),
        _dvo( 0 )
    {
        _transform.header.frame_id = "/world";
        _transform.child_frame_id  = "/rgbd_vo";

        //dynamic_reconfigure::Server<RGBDVOConfig>::CallbackType callback;
        //callback = boost::bind( &RGBDVisualOdometryNode::configCallback, this, _1, _2 );
        //reconfServer_.setCallback( callback );
    }

    RGBDVONode::RGBDVONode( const cvt::Matrix3f& calib ) :
        RGBDSubscriber( calib )
    {
        _transform.header.frame_id = "/world";
        _transform.child_frame_id  = "/rgbd_vo";        
    }

    RGBDVONode::~RGBDVONode()
    {
        if( _dvo != 0 ){
            delete _dvo;
        }
    }

//    void RGBDVONode::configCallback(RGBDVOConfig &config, uint32_t /*level*/ )
//    {
//        config_ = config;
//    }

    void RGBDVONode::imageCallback( const cvt::Image& rgb, const cvt::Image& depth )
    {        
        if( _dvo == 0 ){
            ROS_INFO( "DirectVisualOdometry Created" );
            _dvo = new DirectVisualOdometry( _intrinsics );
        }

        cvt::Image grayf;
        cvt::Image depthf;
        rgb.convert( grayf, cvt::IFormat::GRAY_FLOAT );
        depth.convert( depthf, cvt::IFormat::GRAY_FLOAT );

        cvt::Matrix4f pose = _dvo->computeMotion( grayf, depthf );
        publishPose( pose, _rgbHeader.stamp, "/direct_vo" );
    }

    void RGBDVONode::publishPose( const cvt::Matrix4f& pose,
                                  const ros::Time& t,
                                  const std::string& frameId )
    {
        cvt::Quaternionf q( pose.toMatrix3() );
        _transform.transform.translation.x = pose[ 0 ][ 3 ];
        _transform.transform.translation.y = pose[ 1 ][ 3 ];
        _transform.transform.translation.z = pose[ 2 ][ 3 ];

        _transform.transform.rotation.w = q.w;
        _transform.transform.rotation.x = q.x;
        _transform.transform.rotation.y = q.y;
        _transform.transform.rotation.z = q.z;

        _transform.header.stamp = t;
        _transform.child_frame_id = frameId;
        _tf.sendTransform( _transform );        
    }
}
