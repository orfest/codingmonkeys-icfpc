#include "brain.h"
#include "operations.h"
#include <exception>
#include <iostream>
#include <assert.h>

#include "vector.h"

PortMapping Hohman::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	if (timestep == 0) {
		state = RUNNING;
        r1 = target.minR.length();// sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r2 = target.maxR.length();// sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
		double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v1;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

    int alp = estimateTimeToPerihelionFormula(curEarth, target);
    int i = 1;

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);

		double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		state = COMPLETE;
	}

	timestep++;
	return res;
}

PortMapping CircleToElliptic::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0) {
		state = RUNNING;
		r1 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r2 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
		double delta_v1 = sqrt(MU_CONST / r1) * (sqrt(2 * r2 / (r1 + r2)) - 1);
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v1;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}


double CircleToElliptic::GetTransferTime()
{
	r1 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
	r2 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

	return M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
}

PortMapping EllipticToCircle::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0) {
		state = RUNNING;
		r2 = sqrt(pow(target.minR.x, 2) + pow(target.minR.y, 2));
		r1 = sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));


		double delta_v2 = sqrt(MU_CONST / r2) * (1 - sqrt(2 * r1 / (r1 + r2)));
		Vector prevEarth(Brain::prevInput.find(EARTH_X)->second, Brain::prevInput.find(EARTH_Y)->second);
		Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
		Vector moveDir = prevEarth - curEarth;
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
		clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
		if (!clockwise)
			tangent = -tangent;

		Vector delta = tangent * delta_v2;
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
		transferTime = M_PI * sqrt(pow(r1 + r2, 3) / (8 * MU_CONST));
		state = COMPLETE;
	}
	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}


PortMapping FreeFly::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == ceil(transferTime)) {
		assert(ceil(transferTime) > 1);
		state = COMPLETE;
	}

	timestep++;
	return res;
}

PortMapping FlipDirection::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

    if (timestep == 0){
        state = RUNNING;
        prev = Vector(
            -output.find(EARTH_X)->second,
            -output.find(EARTH_Y)->second
            );
    } else {
        Vector cur(
            -output.find(EARTH_X)->second,
            -output.find(EARTH_Y)->second
            );
        Vector move(cur);
        move -= prev;
	    res[VX_PORT] = -2*move.x;
    	res[VY_PORT] = -2*move.y;
        state = COMPLETE;
    }

	timestep++;
	return res;
}

PortMapping FreeFlyToOpositPoint::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

		Vector position(-output.find(EARTH_X)->second,-output.find(EARTH_Y)->second);
		Vector tar = target.minR;
		tar.normalize();
		position.normalize();
		double dot = Vector::dotProduct(tar,position);
		if (abs(dot+1.0) < 0.0000001){
			state = COMPLETE;
		}
	return res;
}

PortMapping FreeFlyToCertainPoint::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	Vector position(-output.find(EARTH_X)->second,-output.find(EARTH_Y)->second);
    double dist = (position-point).length();
    if (timestep == 0){
        state = RUNNING;
    } else if (timestep > 1){
        if (dist < 1000){
            state = COMPLETE;
        } else if (dist > curDistance && curDistance < prevDistance){
            state = COMPLETE;
        }
    }

    prevDistance = curDistance;
    curDistance = dist;
    timestep++;
	return res;
}

double estimateTimeToPerihelionFormula(const Vector& point, const Orbit& orbit) {
    long double a = (orbit.maxR - orbit.minR).length()*0.5;
    long double ea = (orbit.maxR + orbit.minR).length()*0.5;
    long double e = ea / a;
    long double bb_aa = 1 - e*e;
    long double bb = bb_aa * a*a;
    long double b = sqrt(bb);
    long double p = a*(1-e*e);//b*b/a;
    long double bb_a = bb_aa*a;
    long double r = point.length();
    long double cosphi = (p-r) / (r*e);
    if (cosphi > 1.0) cosphi = 1.0;
    if (cosphi < -1.0) cosphi = -1.0;
    long double phi = acos(cosphi);

    long double A = pow(((1+e)/(1-e)),1);

    long double tnp2 = tan(phi*0.5);
    long double beta = tnp2 / sqrt(A);

    long double pre = 2 / (pow((1-e),2) * sqrt(A));
    long double sum1 = (-1+(1/A))*0.25*sin(2*beta);
    long double sum2 = beta * ( ((1/A) - 1)*0.5 + 1  );
    long double area = pre * (sum1 + sum2) * p * p * 0.5;

    long double total_area = M_PI * a * b; 
    //1927709481952258.5

    long double r1 = orbit.maxR.length();
    long double r2 = orbit.minR.length();
    long double total_time = 2*M_PI*sqrt( pow(r1+r2,3) / (8*MU_CONST) );

    long double res_time = total_time * area / total_area;
    long double fixed_time = total_time * 0.5 - res_time;

    Vector3D to_p(point);
    Vector3D to_apo(orbit.minR);
    Vector3D to_p_apo = Vector3D::crossProduct(to_apo, to_p);
    if ((to_p_apo.z > 0 && !orbit.clockwise) || (to_p_apo.z < 0 && orbit.clockwise)){
        fixed_time = total_time - fixed_time;
    }

    return (double)fixed_time;
}
