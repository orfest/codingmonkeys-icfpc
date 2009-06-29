#include "brain2_2.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B2_2::B2_2(int sn, VM* vm):Brain(sn, vm),startTime(-1), step(0) {}

PortMapping B2_2::_step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (step == 0){
		static Hohman hoh_transfer;
		Orbit target;
		target.minR.x = output.find(EARTH_X)->second;
		target.minR.y = output.find(EARTH_Y)->second;
		target.maxR.x = output.find(TARGET_X)->second - output.find(EARTH_X)->second;
		target.maxR.y = output.find(TARGET_Y)->second - output.find(EARTH_Y)->second;
		hoh_transfer.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&hoh_transfer));
	} else if (step == 1) {
		Orbit target;
		Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
		Vector curTarget(output.find(TARGET_X)->second, output.find(TARGET_Y)->second);
		curTarget += curEarth;
		curEarth.normalize();
		curTarget.normalize();
		Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
		Vector moveDir = prevEarth + curEarth;
		Vector tangent(-curEarth.y, curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		alp = acos(Vector::dotProduct(curEarth, curTarget));
		if ((Vector::crossProduct(curEarth, curTarget) < 0.0 && !clockwise) ||
			(Vector::crossProduct(curEarth, curTarget) > 0.0 && clockwise)){
				alp = 2.0*M_PI - alp;
		}
		alp = alp / (2*M_PI);

		r1 = sqrt(pow(output.find(EARTH_X)->second, 2) + pow(output.find(EARTH_Y)->second, 2));

		double t = alp*2.0*M_PI*sqrt(pow(r1+r1,3.0)/(8.0*MU_CONST));
		double at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
		double at = pow(at3,1.0/3.0);
		double rt = 2.0*at - r1;
		
		//rt = 0.0;
		int i = 1;
		while (rt < EARTH_RADIUS + 1000){
			//rt = r1 * (pow(8.0*pow((alp+i),2),1.0/3) - 1.0);
			t = (alp+i)*2.0*M_PI*sqrt(pow(r1+r1,3.0)/(8.0*MU_CONST));
			at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
			at = pow(at3,1.0/3.0);
			rt = 2.0*at - r1;
			i++;
		}
		target.minR.x = r1;
		target.minR.y = 0;
		target.maxR.x = rt;
		target.maxR.y = 0;
		static CircleToElliptic el1;
		el1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el1));
		static FreeFly wait;
		wait.transferTime = el1.GetTransferTime();
		operation_list.push(static_cast<Operation*>(&wait));
		static EllipticToCircle el2;
		el2.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el2));
	} 

	step++;
	return res;
}

vector<pointF> B2_2::getShipsPositions(const PortMapping& output) const{
    pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
    pointF target( p.first + output.find(TARGET_X)->second, p.second + output.find(TARGET_Y)->second   );
    res.push_back(target);
    return res;
}

int B2_2::getShipsNumber() const{
    return 2;
}

double B2_2::getAngle(const Vector& v) const{
    Vector norm(v);
    norm.normalize();
    double res = acos(norm.x);
    if (norm.y < 0){
        res = -res;
    }
    return res;
}
