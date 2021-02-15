//
// Created by peetcreative on 09.01.21.
//
#include "Esvr2InteractiveElement.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2InteractiveElement2DDef.h"

#include "OgreVector2.h"
#include "OgreColourValue.h"
#include "OgreHlmsUnlit.h"
#include "OgreHlmsUnlitDatablock.h"
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreOverlayManager.h"
#include "Overlay/OgreBorderPanelOverlayElement.h"
#include "Overlay/OgreTextAreaOverlayElement.h"


namespace esvr2
{
    Ogre::ColourValue vectorToColourValue(const std::vector<float> &valVec)
    {
        if(valVec.size() == 4)
        {
            return Ogre::ColourValue(valVec[INDEX_RED], valVec[INDEX_GREEN],
                                     valVec[INDEX_BLUE], valVec[INDEX_ALPHA]);
        }
        return Ogre::ColourValue(0, 0, 0, 0);
    }

    InteractiveElement2D::InteractiveElement2D(
            InteractiveElement2DDefPtr def,
            std::vector<Ogre::IdString> menus,
            //This feels like abusing Ogre..
            Ogre::HlmsUnlit *hlmsUnlit) :
    InteractiveElement(),
    mDefinitionPtr(def),
    mVisibleInMenus(menus)
    {
        BOOST_ASSERT(!mVisibleInMenus.empty());
        mDatablock = dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                hlmsUnlit->getDatablock(Ogre::IdString(mDefinitionPtr->id)));
        if(!mDatablock)
        {
            Ogre::HlmsBlendblock blendBlock = Ogre::HlmsBlendblock();
            blendBlock.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
            mDatablock =
                    dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                            hlmsUnlit->createDatablock(
                                    mDefinitionPtr->id,
                                    mDefinitionPtr->id,
                                    Ogre::HlmsMacroblock(),
                                    blendBlock,
                                    Ogre::HlmsParamVec() ) );
            mDatablock->setUseColour(true);
            mDatablock->setColour( Ogre::ColourValue::Blue);
        }


        Ogre::String overlayId = def->id + menus.at(0).getFriendlyText();
        Ogre::String panelId = overlayId + "Panel";
        Ogre::String textAreaId = overlayId + "TextArea";
        Ogre::String textAreaShadowId = overlayId + "TextAreaShadow";
        Ogre::v1::OverlayManager &overlayManager =
                Ogre::v1::OverlayManager::getSingleton();
        mOverlay = overlayManager.getByName(overlayId);
        if(!mOverlay) {
            mOverlay = overlayManager.create(overlayId);
            mBorderPanel = dynamic_cast<Ogre::v1::BorderPanelOverlayElement *>(
                    overlayManager.createOverlayElement("BorderPanel",
                                                        panelId));
            mBorderPanel->setPosition(
                    def->uvX, def->uvY);
            mBorderPanel->setWidth(def->uvSizeX);
            mBorderPanel->setHeight(def->uvSizeY);
            mBorderPanel->setBorderSize(0.001f);
            mBorderPanel->setBorderMaterialName("ColorWhite");
            mBorderPanel->setMaterialName(mDefinitionPtr->id);

            mTextArea = static_cast<Ogre::v1::TextAreaOverlayElement *>(
                    overlayManager.createOverlayElement("TextArea",
                                                        textAreaId));
            Ogre::v1::TextAreaOverlayElement::Alignment a;
            if (def->alignment == "Left")
                a = Ogre::v1::TextAreaOverlayElement::Left;
            else if (def->alignment == "Right")
                a = Ogre::v1::TextAreaOverlayElement::Right;
            else
                a = Ogre::v1::TextAreaOverlayElement::Center;
            mTextArea->setFontName(def->font);
            mTextArea->setCharHeight(def->fontSize);
            mTextArea->setColour(Ogre::ColourValue::White);
            mTextArea->setAlignment(a);
            mTextArea->setPosition(def->uvSizeX * 0.5f, (def->uvSizeY - def->fontSize)* 0.5f  );
            mBorderPanel->addChild(mTextArea);
            mOverlay->add2D(mBorderPanel);
            mOverlay->setRenderQueueGroup(253);
            if (mDefinitionPtr->alwaysVisible) {
                mDatablock->setColour(
                        vectorToColourValue(mDefinitionPtr->bgColor));
                mOverlay->show();
            } else
                mOverlay->hide();
            mTextArea->setCaption(def->text);
        }
        else
        {
            mBorderPanel = dynamic_cast<Ogre::v1::BorderPanelOverlayElement*>(
                    mOverlay->getChild(panelId));
            mTextArea = static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    mBorderPanel->getChild(textAreaId));
        }
    }

    bool InteractiveElement2D::isOverlaySetup()
    {
        return mOverlay && mBorderPanel && mTextArea;
    }

    bool InteractiveElement2D::isUVinside(Ogre::Vector2 uv)
    {
        Ogre::Vector2 defUv(mDefinitionPtr->uvX, mDefinitionPtr->uvY);
        Ogre::Vector2 a = uv - defUv;
        return 0 <= a.x && a.x <= mDefinitionPtr->uvSizeX &&
                0 <= a.y && a.y <= mDefinitionPtr->uvSizeY;
    }

    Ogre::String InteractiveElement2D::getId()
    {
        return mDefinitionPtr->id;
    }

    bool InteractiveElement2D::setText(Ogre::String text)
    {
        if (!isOverlaySetup())
            return false;
        mTextArea->setCaption(text);
        return true;
    }

    void InteractiveElement2D::setUIState(InteractiveElementStatusType uiStatusType)
    {

        bool show = true;
        Ogre::ColourValue color;
        switch (uiStatusType) {
        case UIS_ACTIVATE:
            color = vectorToColourValue(
                    mDefinitionPtr->bgActiveColor);
            break;
        case UIS_HOVER:
            color = vectorToColourValue(
                    mDefinitionPtr->bgHoverColor);
            break;
        case UIS_VISIBLE:
            color = vectorToColourValue(
                    mDefinitionPtr->bgColor);
            break;
        case UIS_NONE:
            color = vectorToColourValue(mDefinitionPtr->bgColor);
            show = mDefinitionPtr->alwaysVisible;
            break;
        }
        mDatablock->setColour(color);
        if (show)
            mOverlay->show();
        else
            mOverlay->hide();
    }

    bool InteractiveElement2D::isHideOtherOnActive() {
        return mDefinitionPtr->hideOtherOnActive;
    }

    bool InteractiveElement2D::isVisibleOnActive() {
        return mDefinitionPtr->visibleOnActive;
    }

    bool InteractiveElement2D::isVisibleByMenu(Ogre::IdString menu)
    {
        return std::find(mVisibleInMenus.begin(), mVisibleInMenus.end(), menu) != mVisibleInMenus.end();
    }

    bool InteractiveElement2D::isActivatable() {
        return mTogglePressCallback || mHoldCallback || mTogglePressCallback;
    }
}