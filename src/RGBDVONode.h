#ifndef RGBDVONODE_H
#define RGBDVONODE_H

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cvt/math/Matrix.h>
#include <cvt_ros/RGBDSubscriber.h>
#include <DirectVisualOdometry.h>


namespace cvt_ros {

    class RGBDVONode : public RGBDSubscriber
    {
        public:
            RGBDVONode();
            RGBDVONode( const cvt::Matrix3f& calib );
            ~RGBDVONode();

        protected:
            //void configCallback(RGBDVOConfig &config, uint32_t /*level*/ );

            void publishPose( const cvt::Matrix4f& pose,
                              const ros::Time& t,
                              const std::string& frameId );

        private:
            void imageCallback( const cvt::Image& rgb,
                                const cvt::Image& depth );



            //dynamic_reconfigure::Server<RGBDVOConfig> reconfServer_;
            //RGBDVOConfig config_;
            DirectVisualOdometry*           _dvo;
            tf2_ros::TransformBroadcaster   _tf;
            geometry_msgs::TransformStamped	_transform;

    };
}

#endif // RGBDVONODE_H
