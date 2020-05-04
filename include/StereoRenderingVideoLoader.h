
#ifndef _Demo_StereoRenderingLogicSystem_H_
#define _Demo_StereoRenderingLogicSystem_H_

#include "StereoRendering.h"
#include "GameState.h"
namespace Demo {
    class LogicSystem;
    struct GameEntity;
    struct MovableObjectDefinition;

    class VideoLoader : public GameState
    {
        float               mDisplacement;
        GameEntity              *mCubeEntity;
        MovableObjectDefinition *mCubeMoDef;
        VideoInput         mVideoInput;

        LogicSystem         *mLogicSystem;

    public:
        VideoLoader( VideoInput vInput );
        ~VideoLoader();

        void _notifyLogicSystem( LogicSystem *logicSystem )     { mLogicSystem = logicSystem; }

        virtual void initialize(void);
        virtual void deinitialize(void);
        virtual void createScene01(void);
        virtual void update( float timeSinceLast );
    };
}

#endif
