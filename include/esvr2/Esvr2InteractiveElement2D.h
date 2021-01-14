//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_INTERACTIVEELEMENT2D_H
#define ESVR2_INTERACTIVEELEMENT2D_H

#include "Esvr2InteractiveElement2DDef.h"

#include "OgreHlmsUnlitDatablock.h"
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreBorderPanelOverlayElement.h"
#include "Overlay/OgreTextAreaOverlayElement.h"

#include <boost/function.hpp>
#include <memory>

namespace esvr2
{
    typedef enum {
        UIS_NONE,
        UIS_VISIBLE,
        UIS_ACTIVATE
    } UIStatusType;

    class InteractiveElement2D {
    private:
        const boost::function<void(void)> mToggleCallback;
        const boost::function<void(Ogre::uint64)> mHoldCallback;

        Ogre::v1::Overlay *mOverlay = nullptr;
        Ogre::HlmsUnlitDatablock *mDatablock = nullptr;
        Ogre::v1::BorderPanelOverlayElement *mBorderPanel = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextArea = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextAreaShadow = nullptr;
        InteractiveElement2DDefPtr mDefinitionPtr;
    public:

        InteractiveElement2D(InteractiveElement2DDefPtr def,
                             const boost::function<void()> &togglecb,
                             const boost::function<void(Ogre::uint64)> &holdcb,
                             Ogre::HlmsUnlit *hlmsUnlit);

        void activateToggle();
        void activateHold(Ogre::uint64);
        bool isOverlaySetup();
        bool isUVinside(Ogre::Vector2 uv);
        Ogre::String getId();
        bool setText(Ogre::String text);
        void setUIState(UIStatusType uiStatusType, bool hover);
        bool isHideOtherOnActive();
        bool isVisibleOnActive();
    };

    typedef std::shared_ptr <InteractiveElement2D> InteractiveElement2DPtr;
    typedef std::vector <InteractiveElement2DPtr> InteractiveElement2DList;
}

#endif //ESVR2_INTERACTIVEELEMENT2D_H
