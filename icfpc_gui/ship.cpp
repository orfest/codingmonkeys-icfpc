#include "ship.h"

void Ship::pushPosition(const QPointF& p){
    track.push_back(p);
}
