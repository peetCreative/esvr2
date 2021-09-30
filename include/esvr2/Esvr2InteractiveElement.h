//
// Created by peetcreative on 14.02.21.
//

#ifndef ESVR2_ESVR2INTERACTIVEELEMENT_H
#define ESVR2_ESVR2INTERACTIVEELEMENT_H

#include "Ogre.h"
#include <boost/function.hpp>

namespace esvr2
{
    //! \brief Superclass for non/visible elements in the Info/Menu Projectionplane
    /*! To make Elements Interactive register functions for press, hold and releasse
     */
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
        //! \brief calls the registred press callback
        void togglePress();
        //! \brief calls the registred hold callback
        //! @param currentTimeMs  time since the last call
        void hold(Ogre::uint64 currentTimeMs);
        //! \brief calls the registred release callback
        void toggleRelease();
        //! \brief register the press callback
        void setTogglePressFunction(
                const boost::function<void(void)> togglePressCallback);
        //! \brief register the hold callback
        void setHoldFunction(
                const boost::function<void(Ogre::uint64)> holdCallback);
        //! \brief register the release callback
        void setToggleReleaseFunction(
                const boost::function<void(void)> togglePressCallback);
        //! \brief check if this Controller is activable
        /*! @return false if no callbacks are registered
         */
        bool isActivatable();
    };
    typedef std::shared_ptr <InteractiveElement> InteractiveElementPtr;
    typedef std::vector <InteractiveElementPtr> InteractiveElementList;
}

#endif //ESVR2_ESVR2INTERACTIVEELEMENT_H
