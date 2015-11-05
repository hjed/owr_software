/**
 * Uses mouse input to center the oculus on the correct point.
 * Started by: Harry J.E Day
 * Date: 5/11/2015
 */
 
//Qt dependencies
#include <QObject>

#include <rviz/tool.h>

class FocusArm: public rviz::Tool {
    Q_OBJECT
    public:
        FocusArm();
        ~FocusArm();
        
        virtual void onInitalize();
        
        virtual void activate();
        virtual void deactivate();
        
        virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
        
        virtual void load( const rviz::Config& config);
        virtual void save(rviz::Config config) const;
        
        
};
