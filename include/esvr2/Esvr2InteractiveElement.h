//
// Created by peetcreative on 14.02.21.
//

#ifndef ESVR2_ESVR2INTERACTIVEELEMENT_H
#define ESVR2_ESVR2INTERACTIVEELEMENT_H

#include "Ogre.h"
#include <boost/function.hpp>

namespace esvr2
{
    class InteractiveElement
    {
    protected:
        boost::function<void(void)> mTogglePressCallback = (boost::function<void(void)>) 0;
        boost::function<void(Ogre::uint64)> mHoldCallback = (boost::function<void(Ogre::uint64)>) 0;
        boost::function<void(void)> mToggleReleaseCallback = (boost::function<void(void)>) 0;
    public:
        InteractiveElement();
        InteractiveElement(const boost::function<void(void)> togglePressCallback,
                           const boost::function<void(Ogre::uint64)> hold,
                           const boost::function<void(void)> toggleReleaseCallback);
        void togglePress();
        void hold(Ogre::uint64);
        void toggleRelease();
        void setTogglePressFunction(
                const boost::function<void(void)> togglePressCallback);
        void setHoldFunction(
                const boost::function<void(Ogre::uint64)> holdCallback);
        void setToggleReleaseFunction(
                const boost::function<void(void)> togglePressCallback);
    };
    typedef std::shared_ptr <InteractiveElement> InteractiveElementPtr;
    typedef std::vector <InteractiveElementPtr> InteractiveElementList;
}

#endif //ESVR2_ESVR2INTERACTIVEELEMENT_H
