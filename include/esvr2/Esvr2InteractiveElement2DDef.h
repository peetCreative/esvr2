//
// Created by peetcreative on 09.01.21.
//

#ifndef ESVR2_ESVR2INTERACTIVEELEMENT2DDEF_H
#define ESVR2_ESVR2INTERACTIVEELEMENT2DDEF_H

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

    struct InteractiveElement2DDef {
        std::string id = "";
        std::string text = "";
        float uvX = 0;
        float uvY = 0;
        float uvSizeX = 0;
        float uvSizeY = 0;
        float fontSize = 0.05f;
        std::string font = "FreeSans";
        std::vector<float> fontColor = {0,0,0,1};
        bool alwaysVisible = false;
        std::vector<float> bgColor = {1,1,1,0.5};
        std::vector<float> bgHoverColor = {1,0,0,1};
        std::vector<float> bgActiveColor = {1,0,1,1};
        bool visibleOnActive = true;
        bool hideOtherOnActive = false;
    };
    typedef std::shared_ptr<InteractiveElement2DDef> InteractiveElement2DDefPtr;
    typedef std::vector<InteractiveElement2DDefPtr> InteractiveElement2DDefList;

    struct ColorDef {
        std::string name = "";
        std::vector<float> color = {0,0,0,0};
    };
    typedef std::vector<ColorDef> ColorDefList;

    struct InteractiveElementConfig {
        float width = 1.28;
        float height = 0.72;
        std::string bgImg = "";
        std::vector<float> bgColor = {0, 0, 0, 0.01};
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
