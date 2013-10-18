#ifndef DIRECTVISUALODOMETRY_H
#define DIRECTVISUALODOMETRY_H

#include <cvt/vision/rgbdvo/RGBDVisualOdometry.h>
#include <cvt/vision/rgbdvo/IntensityKeyframe.h>
#include <cvt/vision/rgbdvo/KeyframeData.h>
#include <cvt/vision/rgbdvo/Linearizer.h>
#include <cvt/vision/rgbdvo/RGBDWarp.h>

namespace edvo {

    template <class LinearizerType, class OptimizerType>
    class DirectVisualOdometry {
        public:
            typedef typename LinearizerType::AlignDataType                  AlignDataType;
            typedef cvt::IntensityKeyframe<AlignDataType, LinearizerType>   KeyframeType;
            typedef typename OptimizerType::LossFuncType                    LossFuncType;
            typedef cvt::RGBDVisualOdometry<KeyframeType, LossFuncType>     VOType;
            typedef typename VOType::Params                                 Params;

            DirectVisualOdometry( const cvt::Matrix3f& k, const Params& cfg );

            ~DirectVisualOdometry()
            {
                if( _vo )
                    delete _vo;
            }

            const cvt::Matrix4f& computeMotion( const cvt::Image& rgb, const cvt::Image& depth );

            const cvt::Matrix4f& worldToCam() const { return _pose_world_cam; }
            void setWorldToCam( const cvt::Matrix4f& pose ) { _pose_world_cam = pose; }

            OptimizerType* optimizer() { return &_optimizer; }
            void setParams( const Params& params ){ _vo->setParameters( params ); }

        private:
            OptimizerType   _optimizer;
            VOType*         _vo;

            /* camera pose in world frame */
            cvt::Matrix4f   _pose_world_cam;
    };

    template <class LinType, class OptimizerType>
    inline DirectVisualOdometry<LinType, OptimizerType>::DirectVisualOdometry( const cvt::Matrix3f& k, const Params& p ) :
        _vo( 0 )
    {
        _vo = new VOType( &_optimizer, k,  p );
        _pose_world_cam.setIdentity();
    }

    template <class LinType, class OptimizerType>
    inline const cvt::Matrix4f& DirectVisualOdometry<LinType, OptimizerType>::computeMotion( const cvt::Image& gray,
                                                                                             const cvt::Image& depth )
    {
        if( _vo ){
            if( _vo->numKeyframes() == 0 ){
                ROS_INFO( "Adding initial keyframe!" );
                _vo->addNewKeyframe( gray, depth, _pose_world_cam );
                return _pose_world_cam;
            }
            _vo->updatePose( _pose_world_cam, gray, depth );
        }
        return _pose_world_cam;
    }

}

#endif // DIRECTVISUALODOMETRY_H
