
#ifndef _Demo_StereoRenderingLogicSystem_H_
#define _Demo_StereoRenderingLogicSystem_H_

#include "StereoRendering.h"
// 

namespace Demo {
    class LogicSystem;
    struct GameEntity;
    struct MovableObjectDefinition;

    class VideoLoader : public GameState
    {
        float               mDisplacement;
        GameEntity              *mCubeEntity;
        MovableObjectDefinition *mCubeMoDef;

        LogicSystem         *mLogicSystem;

    public:
        LogicGameState();
        ~LogicGameState();

        void _notifyLogicSystem( LogicSystem *logicSystem )     { mLogicSystem = logicSystem; }

        virtual void initialize(void);
        virtual void deinitialize(void);
        virtual void createScene01(void);
        virtual void update( float timeSinceLast );
    };
}

#endif
