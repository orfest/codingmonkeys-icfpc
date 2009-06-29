#include "brain3_3.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "operations.h"

using namespace std;

static const double EPS = 0.1;


B3_3::B3_3(int sn, VM* vm):Brain(sn, vm), step(0){}

PortMapping B3_3::_step(const PortMapping& output){
	/** /
    cout << output.find(EARTH_X)->second << endl;
    cout << output.find(EARTH_Y)->second << endl;
    cout << output.find(FUEL_PORT)->second << endl;
    cout << output.find(SCORE_PORT)->second << endl;
	cout << output.find(TARGET_RADIUS)->second << endl;
	cout << "------------" << endl;
	//*/

    PortMapping res;
	res[SCENARIO_PORT] = 0;
    res[VX_PORT] = 0;
	res[VY_PORT] = 0;
	
	if (step == 0){
		Orbit target;
		target.minR = orbits[1].maxR;
		target.maxR = orbits[0].minR;
		
		static FreeFlyToOpositPoint op1;
		op1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&op1));
		static Hohman hoh1;
		target.minR = orbits[0].minR;
		target.maxR = orbits[1].maxR;
		hoh1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&hoh1));
	} else if (step == 1) {
		Orbit target;
		target.minR = orbits[1].maxR;
		target.maxR = orbits[1].minR;
		Vector pos(output.find(TARGET_X)->second - output.find(EARTH_X)->second,
			output.find(TARGET_Y)->second - output.find(EARTH_Y)->second);
		Vector pos2(-output.find(EARTH_X)->second,
			-output.find(EARTH_Y)->second);
		double alp = estimateTimeToPerihelionFormula(pos,orbits[1]);
		double alp2 = estimateTimeToPerihelionFormula(pos2,orbits[1]);
		double r1 = sqrt(pow(orbits[1].minR.x,2) + pow(orbits[1].minR.y,2));
		double r0 = sqrt(pow(orbits[1].maxR.x,2) + pow(orbits[1].maxR.y,2));

		double T = 2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST));
		alp = M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) + alp;
		alp2 = alp2 - M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST));

		static FreeFly fr1;
		fr1.transferTime = (int)alp2;
		//operation_list.push(static_cast<Operation*>(&fr1));

		alp -= 0.5*alp2;
		double t = alp;// - (alp - 2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)))*0.03;//2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) - 87.5;
		double at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
		double at = pow(at3,1.0/3.0);
		double rt = 2.0*at - r0;
		int i = 1;
		while (rt < EARTH_RADIUS + 1000){
			t = i*2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) + alp;
			at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
			at = pow(at3,1.0/3.0);
			rt = 2.0*at - r0;
			i++;
		}
		target.minR.x = r0;
		target.minR.y = 0;
		target.maxR.x = rt;
		target.maxR.y = 0;
		static CircleToEllipticFast el1;
		el1.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el1));
		static FreeFly wait;
		wait.transferTime = 2.0*M_PI*sqrt(pow(r0+rt,3.0)/(8.0*MU_CONST));
		operation_list.push(static_cast<Operation*>(&wait));
		static EllipticToCircle el2;
		el2.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el2));
		static CircleToEllipticFast el3;
		target.minR.x = r0;
		target.minR.y = 0;
		target.maxR.x = r1;
		target.maxR.y = 0;
		el3.SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el3));
    } 
	for (int I = 0; I < 10; I++)
	if (step == 2+2*I){
		double r0 = sqrt(pow(orbits[1].minR.x,2) + pow(orbits[1].minR.y,2));
		double r1 = sqrt(pow(orbits[1].maxR.x,2) + pow(orbits[1].maxR.y,2));
		double t = 2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST));
		static FreeFly fr[10];
		fr[I].transferTime = t;
		//static FreeFlyToCertainPoint fr;
		//fr.point = orbits[1].minR;
		operation_list.push(static_cast<Operation*>(&fr[I]));
	} else if (step == 3+2*I){
		Orbit target;
		target.minR = orbits[1].minR;
		target.maxR = orbits[1].maxR;
		Vector pos(output.find(TARGET_X)->second - output.find(EARTH_X)->second,
			output.find(TARGET_Y)->second - output.find(EARTH_Y)->second);
		Vector pos2(-output.find(EARTH_X)->second,
			-output.find(EARTH_Y)->second);
		double alp = estimateTimeToPerihelionFormula(pos,orbits[1]);
		double alp2 = estimateTimeToPerihelionFormula(pos2,orbits[1]);
		double r0 = sqrt(pow(orbits[1].minR.x,2) + pow(orbits[1].minR.y,2));
		double r1 = sqrt(pow(orbits[1].maxR.x,2) + pow(orbits[1].maxR.y,2));

		double T = 2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST));
		alp = M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) + alp;
		alp2 = alp2 - M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST));

		static FreeFly fr1;
		fr1.transferTime = alp2;
		//operation_list.push(static_cast<Operation*>(&fr1));

		alp =  0.5 * (alp - T) + T;
		alp -= 0.5*alp2;
		double t = alp;//2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) - 87.5;
		double at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
		double at = pow(at3,1.0/3.0);
		double rt = 2.0*at - r0;
		int i = 1;
		while (rt < EARTH_RADIUS + 1000){
			t = i*2.0*M_PI*sqrt(pow(r0+r1,3.0)/(8.0*MU_CONST)) + alp;
			at3 = MU_CONST*pow(t/(2.0*M_PI),2.0);
			at = pow(at3,1.0/3.0);
			rt = 2.0*at - r0;
			i++;
		}
		static EllipticToCircle el0[10];
		target.minR.x = r0;
		target.minR.y = 0;
		target.maxR.x = r1;
		target.maxR.y = 0;
		el0[I].SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el0[I]));
		target.minR.x = r0;
		target.minR.y = 0;
		target.maxR.x = rt;
		target.maxR.y = 0;
		static CircleToEllipticFast el1[10];
		el1[I].SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el1[I]));
		static FreeFly wait[10];
		wait[I].transferTime = 2.0*M_PI*sqrt(pow(r0+rt,3.0)/(8.0*MU_CONST));
		operation_list.push(static_cast<Operation*>(&wait[I]));
		static EllipticToCircle el2[10];
		el2[I].SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el2[I]));
		static CircleToEllipticFast el3[10];
		target.minR.x = r0;
		target.minR.y = 0;
		target.maxR.x = r1;
		target.maxR.y = 0;
		el3[I].SetTarget(target);
		operation_list.push(static_cast<Operation*>(&el3[I]));
	}

    prevResult = res;
    prevInput = output;
	step++;
    return fuelOveruseFailsafe(output, res);
}

vector<pointF> B3_3::getShipsPositions(const PortMapping& output) const {
	pointF p(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
	pointF target( p.first + output.find(TARGET_X)->second, p.second + output.find(TARGET_Y)->second   );
    res.push_back(target);
    return res;
}

int B3_3::getShipsNumber() const{
    return 2;
}

