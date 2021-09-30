#ifndef ESVR2_COMPONENT_H
#define ESVR2_COMPONENT_H

#include <memory>

namespace esvr2
{
    //! \brief Abstract Component, which can run independently in a thread
    //! if one component is stopped every component is stopped
    /*!
     * \addtogroup Components
     */
    class Component
    {
    public:
        //! \brief called at the beginning
        //! \return success
        virtual bool initialize( void ) {return true;};
        //! \brief called before the Component is stopped
        virtual void deinitialize(void) {};
        //! \brief check if Component wants to stopp the application
        //! \return success
        virtual bool getQuit() {return mQuit;};

        //! \brief check if Component is ready to run
        //! \return success
        virtual bool isReady() {return mReady;};

    protected:
        bool mReady = false;
        bool mQuit = false;

        virtual void quit() {mQuit = true;};
    };
    typedef std::shared_ptr<Component> ComponentPtr;
}

#endif //ESVR2_COMPONENT_H