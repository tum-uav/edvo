#ifndef DIRECTVISUALODOMETRY_H
#define DIRECTVISUALODOMETRY_H

#include <cvt/vision/rgbdvo/RGBDVisualOdometry.h>
#include <cvt/vision/rgbdvo/IntensityKeyframe.h>
#include <cvt/vision/rgbdvo/KeyframeData.h>
#include <cvt/vision/rgbdvo/Linearizer.h>
#include <cvt/vision/rgbdvo/RGBDWarp.h>
#include <cvt/vision/rgbdvo/GNOptimizer.h>

namespace cvt_ros {

    class DirectVisualOdometry {
        public:
            DirectVisualOdometry( const cvt::Matrix3f& k );

            ~DirectVisualOdometry()
            {
                if( _vo )
                    delete _vo;
            }

            cvt::Matrix4f computeMotion( const cvt::Image& rgb, const cvt::Image& depth );

            void setCameraPose( const cvt::Matrix4f& pose ) { _pose_world_cam = pose; }

        private:
            typedef cvt::AffineLightingWarp                                 WarpType;
            typedef cvt::AlignmentData<WarpType>                            AlignDataType;
            typedef cvt::InvCompLinearizer<AlignDataType>                   LinearizerType;
            typedef cvt::IntensityKeyframe<AlignDataType, LinearizerType>   KeyframeType;
            typedef cvt::Huber<float>                                       LossFuncType;
            typedef cvt::RGBDVisualOdometry<KeyframeType, LossFuncType>     VOType;
            typedef cvt::GNOptimizer<AlignDataType, LossFuncType>           OptimizerType;

            OptimizerType   _optimizer;
            VOType*         _vo;

            /* camera pose in world frame */
            cvt::Matrix4f   _pose_world_cam;
    };

    inline DirectVisualOdometry::DirectVisualOdometry( const cvt::Matrix3f& k ) :
        _vo( 0 )
    {
        VOType::Params params;
        params.autoReferenceUpdate = true;
        params.depthScale = 1000.0f;
        params.gradientThreshold = 0.02f;
        params.maxDepth = 10.0f;
        params.minDepth = 0.3f;
        params.maxIters = 12;
        params.maxNumKeyframes = 1;
        params.maxRotationDistance = cvt::Math::deg2Rad( 5.0f );
        params.maxTranslationDistance = 0.25f;
        params.maxSSDSqr = cvt::Math::sqr( 0.2f );
        params.minParameterUpdate = 1e-5f;
        params.minPixelPercentage = 0.7f;        
        params.pyrOctaves = 4;
        params.pyrScale = 0.6f;
        params.selectionPixelPercentage = 0.1;
        params.useInformationSelection = false;

        _vo = new VOType( &_optimizer, k,  params );
        ROS_INFO( "Pyramid Scale: %0.2f: ", _vo->parameters().pyrScale );

        _pose_world_cam.setIdentity();
    }

    inline cvt::Matrix4f DirectVisualOdometry::computeMotion( const cvt::Image& gray,
                                                              const cvt::Image& depth )
    {
        if( _vo && _vo->numKeyframes() == 0 ){
            ROS_INFO( "Adding initial keyframe!" );
            _vo->addNewKeyframe( gray, depth, _pose_world_cam );
            return _pose_world_cam;
        }
        _vo->updatePose( _pose_world_cam, gray, depth );

        return _pose_world_cam;
    }

}

#endif // DIRECTVISUALODOMETRY_H
