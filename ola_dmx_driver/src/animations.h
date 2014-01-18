#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include "animationhost.h"

class FillFade : public Animation {
    public:
        FillFade();

        virtual void renderFrame(QImage* image, const RenderData& data);
        ros::Time firstRender;
        ros::Duration duration;
};


class StarPath : public Animation {
    public:
        StarPath();
        virtual void renderFrame(QImage* image, const RenderData& data);

        ros::Duration duration;

};


#endif  //
