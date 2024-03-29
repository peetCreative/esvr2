//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_ESVR2INTERACTIVEELEMENT2DDEF_H
#define ESVR2_ESVR2INTERACTIVEELEMENT2DDEF_H
#include "Esvr2.h"

#include <memory>
#include <vector>
#include <string>
#include <algorithm>

namespace esvr2
{
    typedef enum {
        INDEX_RED,
        INDEX_GREEN,
        INDEX_BLUE,
        INDEX_ALPHA
    } colorIndexType;

    //! \brief Structure with all visual attributes of a 2D-InteractiveElement
    //! read from the InteractiveElementsDef.yaml
    struct InteractiveElement2DDef {
        std::string id = "";
        std::string text = "";
        Real uvX = 0;
        Real uvY = 0;
        Real uvSizeX = 0;
        Real uvSizeY = 0;
        Real fontSize = 0.05f;
        bool fitFontSize = false;
        std::string font = "FreeSans";
        std::string alignment = "Center";
        std::vector<Real> fontColor = {1,1,1,1};
        bool alwaysVisible = false;
        std::vector<Real> bgColor = {0,0,0,0.7};
        std::vector<Real> bgHoverColor = {1,0,0,1};
        std::vector<Real> bgActiveColor = {1,0,1,1};
        bool visibleOnActive = true;
        bool hideOtherOnActive = false;
    };
    typedef std::shared_ptr<InteractiveElement2DDef> InteractiveElement2DDefPtr;
    typedef std::vector<InteractiveElement2DDefPtr> InteractiveElement2DDefList;

    //! \brief Definition of a Color
    struct ColorDef {
        std::string name = "";
        //! \brief RGBA
        std::vector<Real> color = {0,0,0,0};
    };
    typedef std::vector<ColorDef> ColorDefList;

    //! \brief Structure with the config of the menu projection plane
    //! and the available Defintions for InteractiveElements2D
    struct InteractiveElementConfig {
        Real width = 1.28;
        Real height = 0.72;
        std::string bgImg = "";
        std::vector<Real> bgColor = {0, 0, 0, 0.01};
        ColorDefList colorDefList = {
                {"Transparent", {0,0,0,0}},
                {"ColorRed", {1,0,0,1}},
                {"ColorGreen", {0,1,0,1}},
                {"ColorBlue", {0,0,1,1}},
                {"ColorWhite", {1,1,1,1}},
                {"ColorBlack", {0,0,0,1}},
                {"ColorRedTransparent", {1,0,0,0.5}},
                {"ColorGreenTransparent", {0,1,0,0.5}},
                {"ColorBlueTransparent", {0,0,1,0.5}},
                {"ColorWhiteTransparent", {1,1,1,0.5}}
        };
        InteractiveElement2DDefList defList;

        InteractiveElement2DDefPtr findByName(std::string id)
        {
            InteractiveElement2DDefList::iterator it = find_if(
                    defList.begin(),
                    defList.end(),
                    [&id](const InteractiveElement2DDefPtr& obj)
                    {return obj->id == id;});
            return it != defList.end() ? *it : nullptr;
        }
    };

}

#endif //ESVR2_ESVR2INTERACTIVEELEMENT2DDEF_H
