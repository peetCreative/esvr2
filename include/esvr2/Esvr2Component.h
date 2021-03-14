#ifndef ESVR2_COMPONENT_H
#define ESVR2_COMPONENT_H

#include <memory>

namespace esvr2
{
    class Component
    {
    public:
        virtual bool initialize( void ) {return true;};
        virtual void deinitialize(void) {};
        virtual bool getQuit() {return mQuit;};

        virtual bool isReady() {return mReady;};

    protected:
        bool mReady = false;
        bool mQuit = false;

        virtual void quit() {mQuit = true;};
    };
    typedef std::shared_ptr<Component> ComponentPtr;
}

#endif //ESVR2_COMPONENT_H