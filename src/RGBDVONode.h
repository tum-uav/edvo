#ifndef RGBDVONODE_H
#define RGBDVONODE_H

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cvt/math/Matrix.h>
#include <cvt_ros/RGBDSubscriber.h>
#include <DirectVisualOdometry.h>

#include <edvo/EDVOConfig.h>

namespace edvo {

    template <class OptimizerType, class LinearizerType>
    class RGBDVONode : public cvt_ros::RGBDSubscriber
    {
        public:
            RGBDVONode();
            RGBDVONode( const cvt::Matrix3f& calib );
            ~RGBDVONode();

        protected:
            void publishPose( const cvt::Matrix4f& pose,
                              const ros::Time& t,
                              const std::string& frameId );

        private:
            typedef DirectVisualOdometry<LinearizerType, OptimizerType> DVOType;
            typedef typename DVOType::Params                            VOParams;

            void imageCallback( const cvt::Image& rgb,
                                const cvt::Image& depth );

            void configCallback( EDVOConfig &config, uint32_t /*level*/ );
            void reset( const EDVOConfig& cfg );
            void configToParams( VOParams& params, const EDVOConfig& cfg );

            // dynamic reconfigure
            dynamic_reconfigure::Server<EDVOConfig> _server;
            EDVOConfig                              _config;

            DVOType*                                _dvo;
            tf2_ros::TransformBroadcaster           _tf;
            geometry_msgs::TransformStamped         _transform;

    };

    template <class OptimizerType, class LinearizerType>
    inline RGBDVONode<OptimizerType, LinearizerType>::RGBDVONode() :
        RGBDSubscriber(),
        _dvo( 0 )
    {
        _transform.header.frame_id = "/world";
        _transform.child_frame_id  = "/rgbd_vo";

        _server.setCallback( boost::bind( &RGBDVONode<OptimizerType, LinearizerType>::configCallback, this, _1, _2 ) );
    }

    template <class OptimizerType, class LinearizerType>
    inline RGBDVONode<OptimizerType, LinearizerType>::RGBDVONode( const cvt::Matrix3f& calib ) :
        RGBDSubscriber( calib )
    {
        _transform.header.frame_id = "/world";
        _transform.child_frame_id  = "/rgbd_vo";
    }

    template <class OptimizerType, class LinearizerType>
    inline RGBDVONode<OptimizerType, LinearizerType>::~RGBDVONode()
    {
        if( _dvo != 0 ){
            delete _dvo;
        }
    }

    template <class OptimizerType, class LinearizerType>
    inline void RGBDVONode<OptimizerType, LinearizerType>::imageCallback( const cvt::Image& rgb, const cvt::Image& depth )
    {
        if( _dvo == 0 ){
            reset( _config );
            ROS_INFO( "DirectVisualOdometry instance created" );
        }

        cvt::Image grayf;
        cvt::Image depthf;
        rgb.convert( grayf, cvt::IFormat::GRAY_FLOAT );
        depth.convert( depthf, cvt::IFormat::GRAY_FLOAT );

        cvt::Matrix4f pose = _dvo->computeMotion( grayf, depthf );
        publishPose( pose, _rgbHeader.stamp, "/direct_vo" );
    }

    template <class OptimizerType, class LinearizerType>
    inline void RGBDVONode<OptimizerType, LinearizerType>::publishPose( const cvt::Matrix4f& pose,
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

    template <class OptimizerType, class LinearizerType>
    inline void RGBDVONode<OptimizerType, LinearizerType>::reset( const EDVOConfig& cfg )
    {
        if( _dvo )
            delete _dvo;

        VOParams params;
        configToParams( params, cfg );
        _dvo = new DVOType( _intrinsics, params );

        _dvo->optimizer()->setMinUpdate( cfg.optimizer_min_p_upd );
        _dvo->optimizer()->setMaxIterations( cfg.optimizer_iterations );
        _dvo->optimizer()->setMinPixelPercentage( cfg.ref_upd_min_pix_perc );
        _dvo->optimizer()->setUseRegularization( cfg.optimizer_use_regularizer );
        _dvo->optimizer()->setRegularizationAlpha( cfg.optimizer_reg_alpha );
    }

    template <class OptimizerType, class LinearizerType>
    inline void RGBDVONode<OptimizerType, LinearizerType>::configToParams( VOParams& params, const EDVOConfig& cfg )
    {
        params.autoReferenceUpdate = true;
        params.depthScale = cfg.depth_to_meter;
        params.gradientThreshold = cfg.gradient_threshold;
        params.maxDepth = cfg.reference_max_depth;
        params.minDepth = cfg.reference_min_depth;
        params.maxIters = cfg.optimizer_iterations;
        params.maxNumKeyframes = 1;
        params.maxRotationDistance = cvt::Math::deg2Rad( cfg.ref_upd_max_rot_dist );
        params.maxTranslationDistance = cfg.ref_upd_max_t_dist;
        params.maxSSDSqr = cvt::Math::sqr( cfg.ref_upd_max_ssd );
        params.minParameterUpdate = cfg.optimizer_min_p_upd;
        params.minPixelPercentage = cfg.ref_upd_min_pix_perc;
        params.pyrOctaves = cfg.pyramid_octaves;
        params.pyrScale = cfg.pyramid_scale;
        params.selectionPixelPercentage = cfg.information_selection_thresh;
        params.useInformationSelection = cfg.use_info_select;
    }

    template <class OptimizerType, class LinearizerType>
    inline void RGBDVONode<OptimizerType, LinearizerType>::configCallback( EDVOConfig &config, uint32_t /*level*/ )
    {
        if( _dvo != 0 ){
            // if these parameters change, we need to clear the stored keyframe data
            if( config.depth_to_meter != _config.depth_to_meter ||
                config.pyramid_octaves != _config.pyramid_octaves ||
                config.pyramid_scale != _config.pyramid_scale ){
                cvt::Matrix4f pose = _dvo->worldToCam();
                reset( config );
                _dvo->setWorldToCam( pose );
            }

            _dvo->optimizer()->setMinUpdate( config.optimizer_min_p_upd );
            _dvo->optimizer()->setMaxIterations( config.optimizer_iterations );
            _dvo->optimizer()->setMinPixelPercentage( config.ref_upd_min_pix_perc );
            _dvo->optimizer()->setUseRegularization( config.optimizer_use_regularizer );
            _dvo->optimizer()->setRegularizationAlpha( config.optimizer_reg_alpha );

            VOParams params;
            configToParams( params, config );
            _dvo->setParams( params );

            if( config.reset ){
                reset( config );
                config.reset = false;
            }

        }

        _config = config;
    }
}

#endif // RGBDVONODE_H
