#include "ship.h"

void Ship::pushPosition(const QPointF& p){
    track.push_back(p);
	removeExcessTrackPoints();
}

void Ship::removeExcessTrackPoints(int maxPoints) {
	if (track.size() > maxPoints) {
		QVector<QPointF> newTrack;
		newTrack.reserve(track.size());
		for (int i = 0; i < track.size(); i++) {
			if (!(i & 1))
				newTrack.append(track[i]);
		}
		track = newTrack;
	}
}