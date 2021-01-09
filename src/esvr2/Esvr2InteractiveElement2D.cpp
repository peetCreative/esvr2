//
// Created by peetcreative on 09.01.21.
//
#include "Esvr2InteractiveElement2D.h"

namespace esvr2
{
    InteractiveElement2D::InteractiveElement2D(
            InteractiveElement2DDef def,
            const boost::function<void()> &togglecb,
            const boost::function<void(Ogre::uint64)> &holdcb) :
    mToggleCallback(togglecb),
    mHoldCallback(holdcb),
    mDefinition(def) {};

    //TODO: add visual effects like changing the color
    void InteractiveElement2D::activateToggle() {
        if (mToggleCallback)
            mToggleCallback();
    };

    void InteractiveElement2D::activateHold(Ogre::uint64 msSinceLast) {
        if (mHoldCallback)
            mHoldCallback(msSinceLast);
    };


    bool InteractiveElement2D::isOverlaySetup()
    {
        return mOverlay && mBorderPanel && mTextArea && mTextAreaShadow;
    }

    bool InteractiveElement2D::isUVinside(Ogre::Vector2 uv)
    {
        Ogre::Vector2 a = uv - mDefinition.uv;
        return 0 <= a.x && a.x <= mDefinition.size.x &&
                0 <= a.y && a.y <= mDefinition.size.y;
    }


    bool InteractiveElement2D::setText(Ogre::String text)
    {
        if (!isOverlaySetup())
            return false;
        mTextAreaShadow->setCaption(text);
        mTextArea->setCaption(text);
        return true;
    }
}