#include "starfield.h"

StarInfo::StarInfo () {
    position.setValue(0,0,0);
    velocity.setValue(0,0,0);
    force.setValue(0,0,0);
    mass = 1.0f;
    created = ros::Time::now();
    maxDuration = 5 + (rand() % 10);	// Harcoded. TODO replace?
}

void StarInfo::updatePosition () {
    velocity += force;
    position += velocity;
}

StarfieldState::StarfieldState () { }

StarfieldState::update () {
    tf::Vector3 dist2;
    tf::Vector3 force;
    tf::tfScalar newtons;

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

    foreach (StarInfo si, objects) {
        force.setValue(0,0,0);

        foreach (StarInfo gw, wells) {
            dist2 = si.distance2(gw);

            while (dist2 == 0) {
                si.position[0] += (rand() % 0.1);
                si.position[1] += (rand() % 0.l);

                dist2 = si.distance2(gw);
            }

            // TODO: Hardcoded the minimum distance,
            // support configurable minimum if needed
            if (dist2 < 0.25) {
                dist2 = 0.25;
            }

            newtons = (StarfieldState::G * si.mass * gw.mass) / dist2;
            force += newtons * si.angle(gw);
        }
    }

    foreach (StarInfo si, objects) {
	si.updatePosition();
    }
}
