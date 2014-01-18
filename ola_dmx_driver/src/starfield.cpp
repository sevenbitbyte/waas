#include "starfield.h"
#include <math.h>

StarInfo::StarInfo(ObjectType t, PositionMethod m) {
    position.setValue(0,0,0);
    velocity.setValue(0,0,0);
    force.setValue(0,0,0);
    mass = 1.0f;
    created = ros::Time::now();
    maxDuration = ros::Duration(0, 0);

    radius = 1.0f;
    type = t;
    method = m;
    trackedBlobId = -1;

    if(t == Star){
        maxDuration = maxDuration.fromSec( ((double)(qrand() % 100) / 100.0f) * 5.0f );
    }
}

StarInfo::StarInfo(int id, BlobInfo* blob, ObjectType t, PositionMethod m) {
    position = blob->centroid;
    velocity.setValue(0,0,0);
    force.setValue(0,0,0);
    mass = 0.5f;
    created = ros::Time::now();
    maxDuration = ros::Duration(0, 0);

    radius = 6.0f;
    type = t;
    method = m;
    trackedBlobId = id;
}

void StarInfo::updatePosition () {
    velocity += force;
    position += velocity;
}

bool StarInfo::operator==(const StarInfo& other){
    return ( (this->created == other.created) &&
           (this->force == other.force) &&
           (this->mass == other.mass) &&
           (this->maxDuration == other.maxDuration) &&
           (this->position == other.position) &&
           (this->velocity == other.velocity) );
}

Starfield::Starfield() {
    _gravity = -0.2f;
    _minStars = 3;
    _maxStars = 40;
    _starCount = 0;
    _emitProbability = 0.05f;
    //_bounds = bounds;
}

bool Starfield::isTracked(int blobId){
    return _starsByBlobId.contains(blobId);
}

void Starfield::insertStar(StarInfo* s) {
    _stars.push_back(s);
    _starsByType.insert(s->type, s);
    _starsByMethod.insert(s->method, s);

    if(s->trackedBlobId >= 0){
        _starsByBlobId.insert(s->trackedBlobId, s);
    }

    if(s->type == StarInfo::Star){
        _starCount++;
    }
}

void Starfield::removeStars(QList<StarInfo*> list) {
    foreach(StarInfo* s, list){
        removeStar(s);
    }
}

void Starfield::setBounds(QRectF bounds) {
    _bounds = bounds;
}

void Starfield::removeStar(StarInfo* s) {
    _starsByType.remove(s->type, s);
    _starsByMethod.remove(s->method, s);
    _stars.removeAll(s);

    if(s->type == StarInfo::Star){
        _starCount--;
    }

    if(s->trackedBlobId > 0){
        _starsByBlobId.remove(s->trackedBlobId, s);
    }

    delete s;
}

void Starfield::update(const RenderData& blobs) {
/*
    foreach (StarInfo si, objects) {
	foreach (StarInfo si2, objects) {
	    if (si != si2 && si.dist(si2) < 1) {
		// TODO: these two stars are close
		// consider joining them into a larger
		// star that lives longer and is more
		// massive
	    }
	}
    }
*/

    //Update sensor based positions
    QList<StarInfo*> removeLater;
    QMultiMap<StarInfo::PositionMethod, StarInfo*>::iterator sensorStars = _starsByMethod.find(StarInfo::Sensor);

    while(sensorStars != _starsByMethod.end() && sensorStars.key()==StarInfo::Sensor){
        StarInfo* star = sensorStars.value();
        int sensorId = star->trackedBlobId;

        QMultiMap<int, BlobInfo*>::const_iterator blobIter = blobs.blobs.find(sensorId);

        if(blobIter == blobs.blobs.end()){
            //Delete star
            sensorStars = _starsByMethod.erase(sensorStars);
            removeLater.push_back(star);
            continue;
        }

        star->position = blobIter.value()->centroid;
        sensorStars++;
    }
    removeStars(removeLater);

    //Emit stars
    if(_starCount < _maxStars) {
        bool emitReq = false;

        if(_starCount > _minStars) {
            float v = (float)(qrand()%100) / 100.0f;

            emitReq = (v < _emitProbability);
        }
        else{
            emitReq = true;
        }

        if(emitReq){
            int starCapacity = _maxStars - _starCount;                      //Environment max that can be emitted
            int emitMax = starCapacity * ((float)(qrand()%100) / 100.0f);   //Number that will be emitted now

            qDebug() << "Starfield::update - Emitting up to " << emitMax << " stars ouf of environment capacity " << starCapacity;

            while(emitMax > 0){
                QMultiMap<StarInfo::ObjectType, StarInfo*>::iterator emitterIter = _starsByType.find(StarInfo::Emitter);

                for(emitterIter; emitterIter != _starsByType.end() && emitMax > 0; emitterIter++){
                    StarInfo* s = emitterIter.value();

                    tfScalar range = (s->radius * ((double)(qrand()%100) / 100.0f)) + s->radius;
                    tfScalar angle = 2.0f * M_PI * ((double)(qrand()%100) / 100.0f);

                    tf::Vector3 shift;

                    shift.setX(range * cos(angle));
                    shift.setY(range * sin(angle));
                    shift.setZ(0.0f);

                    StarInfo* newStar = new StarInfo();
                    newStar->position = s->position + shift;

                    insertStar(newStar);
                    emitMax--;
                }
            }
        }
    }


    QMultiMap<StarInfo::PositionMethod, StarInfo*>::iterator physicsStars = _starsByMethod.find(StarInfo::Physics);

    removeLater.clear();

    while(physicsStars != _starsByMethod.end()) {
        tfScalar dist2;
        tfScalar newtons;
        StarInfo* star = physicsStars.value();
        if(star->method != StarInfo::Physics) { break; }

        if(star->maxDuration.toSec() > 0){
            ros::Duration age = ros::Time::now() - star->created;

            //Check if star is too old
            if(age > star->maxDuration){
                //Skip star, schedule delete
                removeLater.push_back(star);

                physicsStars++;
                continue;
            }
        }

        //Check if star is out of bounds
        if(!_bounds.contains( star->position.x(), star->position.y() )){
            //Skip star, schedule delete
            removeLater.push_back(star);

            physicsStars++;
            continue;
        }

        star->force.setValue(0,0,0);


        QList<StarInfo*>::iterator starIter = _stars.begin();
        for(; starIter != _stars.end(); starIter++) {
            StarInfo* well = *starIter;
            if(well->type == StarInfo::Star){ continue; }

            tf::Vector3 d = star->position - well->position;
            double dist2 = d.length();
            dist2 = dist2*dist2;

            // TODO: Hardcoded the minimum distance,
            // support configurable minimum if needed
            if (dist2 < well->radius) {
                dist2=0.01f;
            }

            newtons = (_gravity * star->mass * well->mass) / dist2;

            star->force += newtons * d;
        }

        qDebug() << "Force=" << star->force.length2() << " velocity=" << star->velocity.length2();
        star->updatePosition();
        physicsStars++;
    }

    removeStars(removeLater);
}

QList<StarInfo*> Starfield::getStars() const {
    return _stars;
}

StarSim::StarSim(){
    state = new Starfield();

    StarInfo* mainEmitter = new StarInfo(StarInfo::Emitter, StarInfo::Static);
    mainEmitter->position.setX(16);
    mainEmitter->position.setY(16);

    StarInfo* mainRepulsor = new StarInfo(StarInfo::Repulsor, StarInfo::Static);
    mainRepulsor->position.setX(8);
    mainRepulsor->position.setY(8);

    StarInfo* mainAttractor = new StarInfo(StarInfo::Attractor, StarInfo::Static);
    mainAttractor->position.setX(8);
    mainAttractor->position.setY(24);

    state->insertStar(mainEmitter);
    state->insertStar(mainRepulsor);
    state->insertStar(mainAttractor);
}


void StarSim::renderFrame(QImage* image, const RenderData& data) {
    QRectF bounds = image->rect();
    state->setBounds( bounds );

    //Update physics
    state->update(data);

    QList<StarInfo*> stars = state->getStars();

    QPainter painter;
    painter.begin( image );

    ros::Time now = ros::Time::now();

    foreach(StarInfo* star, stars){

        qreal agePercent = 0.0f;

        if(star->maxDuration.toSec() > 0.0f){
            ros::Duration delta = now - star->created;

            agePercent = delta.toSec() / star->maxDuration.toSec();
        }


        double widthPx = star->radius * 2.0f;

        double centerXPx = star->position.x();
        double centerYPx = star->position.y();

        QRectF bounds(centerXPx - (widthPx/2.0f),
                      centerYPx - (widthPx/2.0f),
                      widthPx,
                      widthPx);
        QBrush fillBrush;

        if(star->type == StarInfo::Star){
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), widthPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, 1.0 - agePercent);

            QColor pink(Qt::yellow);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, pink);

            fillBrush = QBrush( radialGrad );
        }
        else if(star->type == StarInfo::Attractor){
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), widthPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, 1.0 - agePercent);

            QColor pink(Qt::red);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, pink);

            fillBrush = QBrush( radialGrad );
        }
        else if(star->type == StarInfo::Emitter){
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), widthPx);
            QColor white(Qt::white);
            white.setHsvF(0, 0, 1.0 - agePercent);

            QColor pink(Qt::green);

            radialGrad.setColorAt(0, white);
            radialGrad.setColorAt(1.0f, pink);

            fillBrush = QBrush( radialGrad );
        }
        else if(star->type == StarInfo::Repulsor){
            QRadialGradient radialGrad(QPointF(centerXPx,centerYPx), widthPx);
            QColor white(Qt::blue);
            white.setHsvF(0, 0, 1.0);

            QColor pink(Qt::white);

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
