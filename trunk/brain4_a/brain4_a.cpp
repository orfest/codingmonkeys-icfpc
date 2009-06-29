#include "brain4_a.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B4_a::B4_a(int sn, VM* vm):Brain(sn, vm),step(0) {}

PortMapping B4_a::_step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (step == 1){
        Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
        Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
        Vector myMove(prevEarth-curEarth);

        Orbit myOrbit;
        Orbit shipOrbit;
	    Orbit target;

        bool first = true;
        estimateOrbit(myMove, -curEarth, myOrbit.minR, myOrbit.maxR);
        for (int tnum = 6; tnum >= 0; tnum--){

            // assume I'm on a circular

            Vector ship(output.find(TARGETN_X(tnum))->second, output.find(TARGETN_Y(tnum))->second);
            ship -= curEarth;
            Vector prevShip(prevInput.find(TARGETN_X(tnum))->second, prevInput.find(TARGETN_Y(tnum))->second);
            prevShip -= prevEarth;

            Vector shipMove(ship - prevShip);
            estimateOrbit(shipMove, ship, shipOrbit.minR, shipOrbit.maxR);

            target.minR = Vector(shipOrbit.maxR).normalize().operator*=(-myOrbit.minR.length());  // we are on a circular
            target.maxR = shipOrbit.maxR;
            target.clockwise = false;

            FreeFlyToCertainPoint* flyToPoint = new FreeFlyToCertainPoint;
            flyToPoint->point = target.minR;
            operation_list.push(static_cast<Operation*>(flyToPoint));

            CircleToElliptic* cte1 = new CircleToElliptic;
            cte1->SetTarget(target);
		    operation_list.push(static_cast<Operation*>(cte1));

            FreeFly* fly1 = new FreeFly;
            fly1->transferTime = 3;
            operation_list.push(static_cast<Operation*>(fly1));

            if (first){
                FlipDirection* flip = new FlipDirection;
	            operation_list.push(static_cast<Operation*>(flip));

                FreeFly* fly2 = new FreeFly;
                fly2->transferTime = 3;
                operation_list.push(static_cast<Operation*>(fly2));

                first = false;
            }

            double delta_v2;
            double delta_v1;
            {
                double r1 = target.minR.length();
                double r2 = target.maxR.length();
                delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));            
                // fuel from current ellipse to circular
            }
            {
                double r1 = shipOrbit.maxR.length();
                double r2 = shipOrbit.minR.length();
                delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
                // fuel from current circular to ship's ellipse
            }
            delta_v2 -= 1.2*delta_v1;     // save some for binary search

            Accelerate* acc = new Accelerate();
            acc->setDelta(delta_v2);
            operation_list.push(static_cast<Operation*>(acc));

            MeetShip* meet = new MeetShip();
            meet->setShip(tnum);
            operation_list.push(static_cast<Operation*>(meet));

            myOrbit = shipOrbit;
            FreeFlyToCertainPoint* flyToPoint2 = new FreeFlyToCertainPoint;
            flyToPoint2->point = myOrbit.minR;
            operation_list.push(static_cast<Operation*>(flyToPoint2));

            double delta_v3;
            {
                double r1 = myOrbit.minR.length();
                double r2 = myOrbit.maxR.length();
                delta_v3 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));            
                // fuel from current ellipse to circular
            }
            Accelerate* acc = new Accelerate();
            acc->setDelta(delta_v3);
            operation_list.push(static_cast<Operation*>(acc));


            EllipticToCircle* ec1 = new EllipticToCircle;
            target.minR = myOrbit.minR;
            target.maxR = myOrbit.maxR;
            ec1->SetTarget(target);
            operation_list.push(static_cast<Operation*>(ec1));

            myOrbit.minR = -myOrbit.maxR;
        }
	} 

	step++;
	return res;
}

vector<pointF> B4_a::getShipsPositions(const PortMapping& output) const{
    pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
	pointF station( p.first + output.find(STATION_X)->second, p.second + output.find(STATION_Y)->second   );
    res.push_back(station);

	for (int i = 0; i < 11 /* 12 */; i++) {	// in spec there are 12 targets (0..11)
		pointF target( p.first + output.find(TARGETN_X(i))->second, p.second + output.find(TARGETN_Y(i))->second );
		res.push_back(target);
	}

	pointF moon( p.first + output.find(MOON_X)->second, p.second + output.find(MOON_Y)->second   );
    res.push_back(moon);

    return res;
}

int B4_a::getShipsNumber() const{
//    return getShipsPositions().size();
    return 2+9+1+1+1;	// in spec 2+12+1
    //return 2+11+1;	// in spec 2+12+1
}
