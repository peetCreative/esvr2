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
    struct InteractiveElement2DDef {
        std::string id = "";
        std::string text = "";
        float uvX = 0;
        float uvY = 0;
        float uvSizeX = 0;
        float uvSizeY = 0;
        float fontSize = 0.05f;
        std::string font = "FreeSans";
        std::vector<float> fontColor = {1.0, 1.0, 1.0, 1.0};
        std::vector<float> bgHoverColor = {0.8, 0.5, 0.0, 0.3};
        std::vector<float> bgActiveColor = {1.0, 0.0, 0.0, 0.5};
    };
    typedef std::shared_ptr<InteractiveElement2DDef> InteractiveElement2DDefPtr;
    typedef std::vector<InteractiveElement2DDefPtr> InteractiveElement2DDefList;

    struct InteractiveElementConfig {
        float width = 1.28;
        float height = 0.72;
        std::string bgImg = "";
        std::vector<float> bgColor = {0,0,0,0};
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
