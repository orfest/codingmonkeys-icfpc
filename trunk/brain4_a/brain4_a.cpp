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
        int tnum = 1;
		Orbit target;
        Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
//		target.minR = curEarth;
        Vector farShip(output.find(TARGETN_X(tnum))->second, output.find(TARGETN_Y(tnum))->second);
        farShip -= curEarth;
//      target.maxR = farShip;

        double myRadius = curEarth.length();

        Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
        Vector prevFarShip(prevInput.find(TARGETN_X(tnum))->second, prevInput.find(TARGETN_Y(tnum))->second);
        prevFarShip -= prevEarth;

        Vector shipMove(farShip - prevFarShip);
        
        Orbit farShipOrbit;
        estimateOrbit(shipMove, farShip, farShipOrbit.minR, farShipOrbit.maxR);

        target.minR = Vector(farShipOrbit.maxR).normalize().operator*=(-myRadius);
        target.maxR = farShipOrbit.maxR;
        target.clockwise = false;

        static FreeFlyToCertainPoint flyToPoint;
        flyToPoint.point = target.minR;
        operation_list.push(static_cast<Operation*>(&flyToPoint));

        static CircleToElliptic cte1;
        cte1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&cte1));

        static FreeFly fly1;
        fly1.transferTime = 5;
        operation_list.push(static_cast<Operation*>(&fly1));

        static FlipDirection flip;
		operation_list.push(static_cast<Operation*>(&flip));

        static FreeFly fly2;
        fly2.transferTime = 5;
        operation_list.push(static_cast<Operation*>(&fly2));

        double delta_v2;
        double delta_v1;
        {
            double r1 = target.minR.length();
            double r2 = target.maxR.length();
            delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
        }
        {
            double r1 = farShipOrbit.maxR.length();
            double r2 = farShipOrbit.minR.length();
            delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
        }
        delta_v2 -= delta_v1;

        static Accelerate acc;
        acc.setDelta(delta_v2);
        operation_list.push(static_cast<Operation*>(&acc));

        static MeetShip meet;
        meet.setShip(tnum);
        operation_list.push(static_cast<Operation*>(&meet));

	//} else if (!start) {
	//	Orbit target;
	//	Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	//	//Vector curTarget(output.find(TARGETN_X(did))->second, output.find(TARGETN_Y(did))->second);
 //       Vector curTarget(output.find(MOON_X)->second, output.find(MOON_Y)->second);
	//	curTarget += curEarth;
	//	curEarth.normalize();
	//	curTarget.normalize();
	//	Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
	//	Vector moveDir = prevEarth + curEarth;
	//	Vector tangent(-curEarth.y, curEarth.x);
	//	tangent.normalize();
	//	clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
	//	alp = acos(Vector::dotProduct(curEarth, curTarget));
	//	if ((Vector::crossProduct(curEarth, curTarget) < 0.0 && !clockwise) ||
	//		(Vector::crossProduct(curEarth, curTarget) > 0.0 && clockwise)){
	//			alp = 2.0*M_PI - alp;
	//	}
	//	alp = alp / (2*M_PI);
	//	r1 = sqrt(pow(output.find(EARTH_X)->second, 2) + pow(output.find(EARTH_Y)->second, 2));
	//	rt = 0.0;
	//	int i = 0;
	//	while (rt < EARTH_RADIUS + 1000){
	//		rt = r1 * (pow(8.0*pow((alp+i),2),1.0/3) - 1.0);
	//		i++;
	//	}
	//	target.minR.x = r1;
	//	target.minR.y = 0;
	//	target.maxR.x = rt;
	//	target.maxR.y = 0;
	//	static CircleToElliptic el1;
	//	el1.SetTarget(target);
	//	operation_list.push(static_cast<Operation*>(&el1));
	//	static FreeFly wait;
	//	wait.transferTime = el1.GetTransferTime();
	//	operation_list.push(static_cast<Operation*>(&wait));
	//	static EllipticToCircle el2;
	//	el2.SetTarget(target);
	//	operation_list.push(static_cast<Operation*>(&el2));
 //       start = !start;
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
