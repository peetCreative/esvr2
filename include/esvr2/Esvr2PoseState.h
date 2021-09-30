#ifndef _Esvr2_POSESTATE_H_
#define _Esvr2_POSESTATE_H_

#include "Esvr2.h"
#include "Esvr2Component.h"

namespace esvr2 {
    /*! Class to provide the externally
     * tracked pose of the laparoscope cameras.
     * It is by now only used for debugging.
     *
     * \addtogroup Components
     */
    class PoseState: virtual public Component {
    protected:
        RealArray16 mPose;

        RealArray3 mPosition;
        RealArray4 mOrientation;
        //! internal fct setPose using R3 Vector and Quaternion
        void setPose(RealArray3 position, RealArray4 orientation);

        bool mValidPose;

    public:
        PoseState();
        ~PoseState();

        //! \brief 4x4 Transformationmatrix R+t
        virtual RealArray16 getPose(void);
        //! \brief get R3 Vector relativ to defined origin
        virtual RealArray3 getPosition(void);
        //! \brief get Quaternion relativ to defined origin
        virtual RealArray4 getOrientation(void);
        //! \brief check if there is a valid and current Pose available
        bool validPose();

    };
    typedef std::shared_ptr<PoseState> PoseStatePtr;
}

#endif
