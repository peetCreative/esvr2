//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_INTERACTIVEELEMENT2D_H
#define ESVR2_INTERACTIVEELEMENT2D_H

#include "Esvr2InteractiveElement.h"
#include "Esvr2InteractiveElement2DDef.h"

#include "OgreHlmsUnlitDatablock.h"
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreBorderPanelOverlayElement.h"
#include "Overlay/OgreTextAreaOverlayElement.h"

#include <boost/function.hpp>
#include <memory>

namespace esvr2
{
    enum InteractiveElementStatusType {
        UIS_NONE,
        UIS_VISIBLE,
        UIS_HOVER,
        UIS_ACTIVATE
    } ;

    class InteractiveElement2D : public InteractiveElement {
    private:
        Ogre::v1::Overlay *mOverlay = nullptr;
        Ogre::HlmsUnlitDatablock *mDatablock = nullptr;
        Ogre::v1::BorderPanelOverlayElement *mBorderPanel = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextArea = nullptr;
        InteractiveElement2DDefPtr mDefinitionPtr;
        std::vector<Ogre::IdString> mVisibleInMenus;
    public:

        InteractiveElement2D(InteractiveElement2DDefPtr def,
                             std::vector<Ogre::IdString> visibleMenus,
                             Ogre::HlmsUnlit *hlmsUnlit);

        bool isOverlaySetup();
        bool isUVinside(Ogre::Vector2 uv);
        Ogre::String getId();
        bool setText(Ogre::String text);
        void setUIState(InteractiveElementStatusType uiStatusType);
        bool isHideOtherOnActive();
        bool isVisibleOnActive();
        bool isVisibleByMenu(Ogre::IdString menu);
        bool isActivatable();
    };
    typedef std::shared_ptr <InteractiveElement2D> InteractiveElement2DPtr;
    typedef std::vector <InteractiveElement2DPtr> InteractiveElement2DList;
}

#endif //ESVR2_INTERACTIVEELEMENT2D_H
