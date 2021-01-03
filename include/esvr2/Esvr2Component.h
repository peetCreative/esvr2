#ifndef ESVR2_COMPONENT_H
#define ESVR2_COMPONENT_H

namespace esvr2
{
    class Component
    {
    public:
        virtual bool initialize( void ) = 0;
        virtual void deinitialize(void) = 0;
        virtual void update( ) {};
        virtual bool getQuit() {return mQuit;};

        virtual bool isReady() {return mReady;};

    protected:
        bool mReady = false;
        bool mQuit = false;

        void quit() {mQuit = true;};
    };

}

#endif //ESVR2_COMPONENT_H