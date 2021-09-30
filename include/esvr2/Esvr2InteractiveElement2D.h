//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_INTERACTIVEELEMENT2D_H
#define ESVR2_INTERACTIVEELEMENT2D_H

#include "Esvr2InteractiveElement.h"
#include "Esvr2InteractiveElement2DDef.h"

#include "OgreHlmsUnlitDatablock.h"
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgrePanelOverlayElement.h"
#include "Overlay/OgreTextAreaOverlayElement.h"

#include <boost/function.hpp>
#include <memory>

namespace esvr2
{
    //! \brief visiblity/activation state
    enum InteractiveElementStatusType {
        UIS_NONE /*! Invalid*/,
        UIS_VISIBLE /*! It's currently visible*/,
        UIS_HOVER /*! highlighted but not activated*/,
        UIS_ACTIVATE  //! highlighted but not activated
    } ;

    //! \brief InteractiveElement which can be visualized in the menu/InfoScreen
    /*! It's implemented by a Ogre rectengular 2D-Overlay,
     * The element can change it's background color and text
     */
    class InteractiveElement2D : public InteractiveElement {
    private:
        Ogre::v1::Overlay *mOverlay = nullptr;
        Ogre::HlmsUnlitDatablock *mDatablock = nullptr;
        Ogre::v1::PanelOverlayElement *mPanel = nullptr;
        Ogre::v1::TextAreaOverlayElement *mTextArea = nullptr;
        InteractiveElement2DDefPtr mDefinitionPtr;
        std::vector<Ogre::IdString> mVisibleInMenus;
    public:

        InteractiveElement2D(InteractiveElement2DDefPtr def,
                             std::vector<Ogre::IdString> visibleMenus,
                             Ogre::HlmsUnlit *hlmsUnlit);
        //! \brief Check if the 2D Element is ready
        bool isOverlaySetup();
        //! \brief calculate if a given uv coordinate is inside the 2D element
        bool isUVinside(Ogre::Vector2 uv);
        //! \brief get the Id of the 2D-Element
        Ogre::String getId();
        //! \brief changes the Text in the UI
        bool setText(Ogre::String text);
        //! \brief set visiblity/activation state of the
        void setUIState(InteractiveElementStatusType uiStatusType);
        //! \brief if other 2D-Elements should be visible while hold
        bool isHideOtherOnActive();
        //! \brief if THIS 2D-Elements should be visible while hold
        bool isVisibleOnActive();
        //! \brief lookup if it visible in the given menuId
        /*!
         * @param menu lookup menuId
         * @return found
         */
        bool isVisibleByMenu(Ogre::IdString menu);
    };
    typedef std::shared_ptr <InteractiveElement2D> InteractiveElement2DPtr;
    typedef std::vector <InteractiveElement2DPtr> InteractiveElement2DList;
}

#endif //ESVR2_INTERACTIVEELEMENT2D_H
