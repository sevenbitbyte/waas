#include "animations.h"

FillFade::FillFade() {
    firstRender = ros::Time::now();
    duration = ros::Duration(5);
}

void FillFade::renderFrame(QImage* image, const RenderData& data) {
    ros::Duration delta = data.timestamp - firstRender;

    double durationDelta = (double) (delta.toNSec() % duration.toNSec());
    double position = durationDelta / (double) duration.toNSec();

    QColor color = QColor::fromHsvF( position, 0.8, 0.3 );

    image->fill(color);
}


StarPath::StarPath() {
    duration = ros::Duration(1.5);
}

void StarPath::renderFrame(QImage* image, const RenderData& data) {

    QPainter painter;
    painter.begin( image );

    foreach(BlobInfo* blob, data.blobs){
        ros::Duration delta = data.timestamp - blob->timestamp;

        double durationDelta = (double) (delta.toNSec() % duration.toNSec());
        double position = durationDelta / (double) duration.toNSec();

        if(delta > duration){
            continue;
        }

        double widthPx = blob->bounds.x();
        double depthPx = blob->bounds.y();

        double centerXPx = blob->centroid.x();
        double centerYPx = blob->centroid.y();

        double radiusPx = qMax(widthPx, depthPx);
        double radius = qMax(blob->realDimensions.x(), blob->realDimensions.y());

        QRectF bounds(centerXPx - (widthPx/2.0f),
                      centerYPx - (depthPx/2.0f),
                      radiusPx,
                      radiusPx);
        QBrush fillBrush;

        if(radius > 0.75f){
            QConicalGradient conicalGrad(centerXPx,centerYPx, position);
            conicalGrad.setColorAt(0, Qt::red);
            conicalGrad.setColorAt(90.0/360.0, Qt::green);
            conicalGrad.setColorAt(180.0/360.0, Qt::blue);
            conicalGrad.setColorAt(270.0/360.0, Qt::magenta);
            conicalGrad.setColorAt(360.0/360.0, Qt::yellow);

            conicalGrad.setAngle();

            fillBrush = QBrush( conicalGrad );

        }
        else {
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), radiusPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, 1.0-position);

            QColor pink(Qt::magenta);
            pink.setAlphaF(position);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, pink);

            fillBrush = QBrush( radialGrad );
        }

        QPainterPath fillPath;

        fillPath.addEllipse(bounds);

        painter.fillPath(fillPath, fillBrush);


    }
    painter.end();
}


StarSim::StarSim()
    : maxObj(100)
{

}

void StarSim::renderFrame(QImage *image, const RenderData &data) {

    ros::Time now;

    //Clean up old objects
    QList<StarInfo>::iterator objectIter = state.objects.begin();
    while(objectIter != state.objects.end()){
        if (objectIter->position.x() < image->width() || objectIter->position.x() > image->width() ||
            objectIter->position.y() < image->height() || objectIter->position.y() > image->height() ||
            now > (objectIter->created + ros::Duration(objectIter->maxDuration) )) {

            objectIter = state.objects.erase(objectIter);
        }
        else{
            objectIter++;
        }
    }



        /*
        well.position.setValues(blob->centroid.x(), blob->centroid.y(), 0);
        state.wells.push_back(well);

        double widthPx = blob->bounds.x();
        double depthPx = blob->bounds.y();

        double radiusPx = qMax(widthPx, depthPx);

        if (radiusPx > 5.5) {
            if (sf.objects.size() < maxObj) {
                // Create a new star with random values
                StarInfo star;

                star.position[0] = blob->centroid.x() + 1 - (rand()%2);
                star.position[1] = blob->centroid.y() + 1 - (rand()%2);

                double mag = rand() % 5.0f;
                double dir = (rand() % 1) * (2 * 3.1415 / 180);

                star.velocity[0] = cos(dir) * mag;
                star.velocity[1] = sin(dir) * mag;

                state.objects.push_back(star);
            }
        }*/
    //}

    /*state.update();

    QPainter painter;
    painter.begin(image);

    // TODO: Add the painting logic



    painter.end();*/
}
