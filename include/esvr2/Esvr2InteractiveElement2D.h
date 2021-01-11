//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_INTERACTIVEELEMENT2D_H
#define ESVR2_INTERACTIVEELEMENT2D_H

#include "Esvr2InteractiveElement2DDef.h"

#include <OgrePanelOverlayElement.h>
#include <boost/function.hpp>
#include <OgreBorderPanelOverlayElement.h>
#include "Overlay/OgreTextAreaOverlayElement.h"

#include <memory>

namespace esvr2
{
    class InteractiveElement2D {
    private:
        const boost::function<void(void)> mToggleCallback;
        const boost::function<void(Ogre::uint64)> mHoldCallback;
    public:
        Ogre::v1::Overlay *mOverlay = nullptr;
        Ogre::v1::BorderPanelOverlayElement *mBorderPanel = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextArea = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextAreaShadow = nullptr;
        InteractiveElement2DDefPtr mDefinitionPtr;


        InteractiveElement2D(InteractiveElement2DDefPtr def,
                             const boost::function<void()> &togglecb,
                             const boost::function<void(Ogre::uint64)> &holdcb);

        void activateToggle();
        void activateHold(Ogre::uint64);
        bool isOverlaySetup();
        bool isUVinside(Ogre::Vector2 uv);
        bool setText(Ogre::String text);
    };

    typedef std::shared_ptr <InteractiveElement2D> InteractiveElement2DPtr;
    typedef std::vector <InteractiveElement2DPtr> InteractiveElement2DList;
}

#endif //ESVR2_INTERACTIVEELEMENT2D_H
