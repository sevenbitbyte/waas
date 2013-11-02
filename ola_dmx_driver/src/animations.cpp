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

        QRectF bounds(centerXPx - (widthPx/2.0f),
                      centerYPx - (depthPx/2.0f),
                      radiusPx,
                      radiusPx);
        QBrush fillBrush;

        if(radiusPx > 5.5f){
            QConicalGradient conicalGrad(centerXPx,centerYPx, 0);
            conicalGrad.setColorAt(0, Qt::red);
            conicalGrad.setColorAt(90.0/360.0, Qt::green);
            conicalGrad.setColorAt(180.0/360.0, Qt::blue);
            conicalGrad.setColorAt(270.0/360.0, Qt::magenta);
            conicalGrad.setColorAt(360.0/360.0, Qt::yellow);

            fillBrush = QBrush( conicalGrad );

        }
        else {
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), radiusPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, position);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, Qt::black);

            fillBrush = QBrush( radialGrad );
        }

        QPainterPath fillPath;

        fillPath.addEllipse(bounds);

        painter.fillPath(fillPath, fillBrush);


    }
    painter.end();
}

StarSim::StarSim() { }

void StarSim::renderFrame(QImage *image, const RenderData &data) {

    ros::Time now;
    vector<StarInfo>::iterator siitr;

    QPainter painter;
    painter.begin(image);

    siitr = objects.begin();

    while(siitr != objects.end()) {
        StarInfo si = *siitr;

        if (si.position[0] < 0 || si.position[0] > 30 ||
            si.position[1] < 0 || si.position[1] > 30 ||
            now > (st.created + si.maxDuration)) {
            objects.erase(siitr++);
        }

        siitr++;
    }

    wells = vector<StarInfo>();

    foreach(BlobInfo* blob, data.blobs) {
        StarInfo well;
        well.position.setValues(blob->centroid.x(), blob->centroid.y(), 0);
        wells.push_back(well);

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

                sf.objects.push_back(star);
            }
        }
    }

    sf.wells = wells;

    sf.update();

    // TODO: Add the painting logic

    painter.end();
}
