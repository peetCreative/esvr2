//
// Created by peetcreative on 09.01.21.
//
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
            const boost::function<void()> &togglecb,
            const boost::function<void(Ogre::uint64)> &holdcb,
            Ogre::IdString menu,
            //This feels like abusing Ogre..
            Ogre::HlmsUnlit *hlmsUnlit) :
    mToggleCallback(togglecb),
    mHoldCallback(holdcb),
    mDefinitionPtr(def),
    mVisibleInMenu(menu)
    {
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


        Ogre::String overlayId = def->id;
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
            mTextArea->setFontName(def->font);
            mTextArea->setCharHeight(def->fontSize);
            mTextArea->setColour(Ogre::ColourValue::White);
            //        mTextArea->setPosition(0.0f, 0.0f );
            mTextAreaShadow = dynamic_cast<Ogre::v1::TextAreaOverlayElement *>(
                    overlayManager.createOverlayElement("TextArea",
                                                        textAreaShadowId));
            mTextAreaShadow->setFontName(def->font);
            mTextAreaShadow->setCharHeight(def->fontSize);
            mTextAreaShadow->setColour(Ogre::ColourValue::Black);
            //        mTextAreaShadow->setMaterialName("White");
            mTextAreaShadow->setPosition(0.002f, 0.002f);

            mBorderPanel->addChild(mTextAreaShadow);
            mBorderPanel->addChild(mTextArea);
            mOverlay->add2D(mBorderPanel);
            mOverlay->setRenderQueueGroup(253);
            if (mDefinitionPtr->alwaysVisible) {
                mDatablock->setColour(
                        vectorToColourValue(mDefinitionPtr->bgColor));
                mOverlay->show();
            } else
                mOverlay->hide();
            mTextAreaShadow->setCaption(def->text);
            mTextArea->setCaption(def->text);
        }
        else
        {
            mBorderPanel = dynamic_cast<Ogre::v1::BorderPanelOverlayElement*>(
                    mOverlay->getChild(panelId));
            mTextArea = static_cast<Ogre::v1::TextAreaOverlayElement*>(
                    mBorderPanel->getChild(textAreaId));
            mTextAreaShadow = dynamic_cast<Ogre::v1::TextAreaOverlayElement*>(
                    mBorderPanel->getChild(textAreaShadowId));
        }
    };

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
        mTextAreaShadow->setCaption(text);
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
        return mVisibleInMenu == menu;
    }

}