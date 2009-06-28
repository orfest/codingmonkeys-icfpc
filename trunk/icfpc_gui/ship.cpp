#include "ship.h"
#include <math.h>

void Ship::pushPosition(const QPointF& p){
    track.push_back(p);
	removeExcessTrackPoints(maxPointsPerShip);
}

void Ship::removeExcessTrackPoints(int maxPoints) {
	if (track.size() > maxPoints) {
		bool sweepAll = (sweepedPointsUpTo >= track.size() * 0.9);
		doSweep();
		if (sweepAll) {
			if (!(numSweeps & (numSweeps - 1)))
				sweepedPointsUpTo = 0;
			else
				sweepedPointsUpTo = track.size() * 0.5;
			doSweep();
			numSweeps++;
		}
	}
}

void Ship::doSweep() {
	QVector<QPointF> newTrack;
	newTrack.reserve(track.size());
	for (int i = 0; i < track.size(); i++) {
		if (i < sweepedPointsUpTo ||  !(i & 1))
			newTrack.append(track[i]);
	}
	track = newTrack;

	sweepedPointsUpTo = track.size();
}
