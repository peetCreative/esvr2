//
// Created by peetcreative on 14.02.21.
//

#include "Esvr2InteractiveElement.h"

#include <boost/function.hpp>

namespace esvr2
{
    InteractiveElement::InteractiveElement() {}
    InteractiveElement::InteractiveElement(
            const boost::function<void(void)> togglePressCallback,
            const boost::function<void(Ogre::uint64)> holdCallback = (boost::function<void(void)>) 0,
            const boost::function<void(void)> toggleReleaseCallback = (boost::function<void(void)>) 0):
        mTogglePressCallback(togglePressCallback),
        mHoldCallback(holdCallback),
        mToggleReleaseCallback(toggleReleaseCallback)
    {}

    void InteractiveElement::togglePress() {
        if (mTogglePressCallback)
            mTogglePressCallback();
    }

    void InteractiveElement::toggleRelease() {
        if (mToggleReleaseCallback)
            mToggleReleaseCallback();
    }

    void InteractiveElement::hold(Ogre::uint64 currentTimeMs) {
        if (mHoldCallback)
            mHoldCallback(currentTimeMs);
    }

    void InteractiveElement::setTogglePressFunction(
            const boost::function<void(void)> togglePressCallback)
    {
        mTogglePressCallback = togglePressCallback;
    }

    void InteractiveElement::setHoldFunction(
            const boost::function<void(Ogre::uint64)> holdCallback)
    {
        mHoldCallback = holdCallback;
    }

    void InteractiveElement::setToggleReleaseFunction(
            const boost::function<void(void)> toggleReleaseCallback)
    {
        mToggleReleaseCallback = toggleReleaseCallback;
    }

    bool InteractiveElement::isActivatable() {
        return mTogglePressCallback || mHoldCallback || mTogglePressCallback;
    }
}
