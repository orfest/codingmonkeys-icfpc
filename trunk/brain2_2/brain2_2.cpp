#include "brain2_2.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B2_2::B2_2(int sn, VM* vm):Brain(sn, vm),startTime(-1), step(0) {}

PortMapping B2_2::_step(const PortMapping& output){
	///** /
 //   cout << output.find(EARTH_X)->second << endl;
 //   cout << output.find(EARTH_Y)->second << endl;
 //   cout << output.find(FUEL_PORT)->second << endl;
 //   cout << output.find(SCORE_PORT)->second << endl;
	//cout << output.find(TARGET_RADIUS)->second << endl;
	//cout << "------------" << endl;
	////*/

 //   PortMapping res;
	//res[SCENARIO_PORT] = 0;
 //   res[VX_PORT] = 0;
	//res[VY_PORT] = 0;

 //   if (timestep == 0){
 //       assert(output.empty());
 //       res[SCENARIO_PORT] = Brain::scenarioNumber;
	//} else if (timestep == 1) {
	//	r1 = sqrt(pow(output.find(EARTH_X)->second, 2) + pow(output.find(EARTH_Y)->second, 2));
	//	r2 = sqrt(pow(output.find(EARTH_X)->second - output.find(TARGET_X)->second, 2) +
	//		pow(output.find(EARTH_Y)->second - output.find(TARGET_Y)->second, 2));
	//	transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8.0*MU_CONST));
	//} else	if (timestep == 2) {
	//	double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
	//	Vector prevEarth(prevInput.find(EARTH_X)->second, prevInput.find(EARTH_Y)->second);
	//	Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
	//	Vector moveDir = prevEarth - curEarth;
	//	Vector tangent(curEarth.y, -curEarth.x);
	//	tangent.normalize();
	//	clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
	//	if (!clockwise)
	//		tangent = -tangent;

	//	Vector delta = tangent * delta_v1;
	//	res[VX_PORT] = delta.x;
	//	res[VY_PORT] = delta.y;
	//} else if (timestep == ceil(2 + transferTime)) {
	//	assert(ceil(2 + transferTime) > 1);

	//	double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
	//	Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
	//	Vector tangent(curEarth.y, -curEarth.x);
	//	tangent.normalize();
	//	if (!clockwise)
	//		tangent = -tangent;

	//	Vector delta = tangent * delta_v2;
	//	res[VX_PORT] = delta.x;
	//	res[VY_PORT] = delta.y;
	//	secondparttime = ceil(2 + transferTime);
	//} else if (timestep == secondparttime + 1){
	//	Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	//	Vector curTarget(output.find(TARGET_X)->second, output.find(TARGET_Y)->second);
	//	curTarget += curEarth;
	//	curEarth.normalize();
	//	curTarget.normalize();
	//	alp = acos(Vector::dotProduct(curEarth, curTarget));
	//	if ((Vector::crossProduct(curEarth, curTarget) < 0.0 && !clockwise) ||
	//		(Vector::crossProduct(curEarth, curTarget) > 0.0 && clockwise)){
	//		alp = 2.0*M_PI - alp;
	//	}
	//	alp = alp / (2*M_PI);
	//	rt = r2 * (pow(8.0*pow((alp+1.0),2),1.0/3) - 1.0);
	//	
	//	double delta_v = sqrt(2.0*MU_CONST / r2 - 2.0*MU_CONST/(r2+rt)) - sqrt(MU_CONST/r2);
	//	Vector tangent(-curEarth.y, curEarth.x);
	//	tangent.normalize();
	//	if (!clockwise)
	//		tangent = -tangent;

	//	Vector delta = tangent * delta_v;
	//	res[VX_PORT] = delta.x;
	//	res[VY_PORT] = delta.y;
	//	transferTime2 = 2.0*M_PI*sqrt(pow(r2+rt,3)/(8.0*MU_CONST));
	//}else if (timestep == ceil(secondparttime + 1 + transferTime2)){
	//	double delta_v = sqrt(2.0*MU_CONST / r2 - 2.0*MU_CONST/(r2+rt)) - sqrt(MU_CONST/r2);
	//	Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
	//	Vector tangent(curEarth.y, -curEarth.x);
	//	tangent.normalize();
	//	if (!clockwise)
	//		tangent = -tangent;

	//	Vector delta = -tangent * delta_v;
	//	res[VX_PORT] = delta.x;
	//	res[VY_PORT] = delta.y;
	//}


 //   prevResult = res;
 //   prevInput = output;
	//timestep++;
 //   return fuelOveruseFailsafe(output, res);

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
		rt = r1 * (pow(8.0*pow((alp+1.0),2),1.0/3) - 1.0);
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
