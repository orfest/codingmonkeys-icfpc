#include "brain.h"
#include "operations.h"
#include <exception>
#include <iostream>
#include <assert.h>

#include "vector.h"
#include "vm.h"
#include "executer.h"

using namespace std;

PortMapping Hohman::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	Vector curEarth(output.find(EARTH_X)->second, output.find(EARTH_Y)->second);
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
        std::cout << "Circle To Elliptic" << target.minR.length() << " : " << target.maxR.length() << endl;
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

PortMapping CircleToEllipticFast::step(const PortMapping& output){
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
        std::cout << "Elliptic to Circle : " << target.minR.length() << " : " << target.maxR.length() << endl;
		state = RUNNING;
		r2 = target.minR.length();
		r1 = target.maxR.length();//sqrt(pow(target.maxR.x, 2) + pow(target.maxR.y, 2));

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

    if (timestep == 0){
        std::cout << "Free Fly for " << transferTime << endl;
    }

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
        std::cout << "Flip Direction" << endl;
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

PortMapping Accelerate::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

    if (timestep == 0){
        state = RUNNING;
        std::cout << "Accelerate: delta: " << delta << endl;
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
        move.normalize();
        move *= delta;
	    res[VX_PORT] = move.x;
    	res[VY_PORT] = move.y;
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
        std::cout << "Free Fly to Certain Point: X: " << point.x << " Y: " << point.y << endl;
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

PortMapping MeetShip::step(const PortMapping& output){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

    if (timestep == 0){
        std::cout << "Meet Ship " << ship << endl;
        state = RUNNING;
    }

    Vector pos(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
    Vector toShip(output.find(TARGETN_X(ship))->second, output.find(TARGETN_Y(ship))->second);

    double dist = toShip.length();
    if (dist < 1000){
        state = COMPLETE;
        searchSuccessful = true;
    }

    if (!searchSuccessful && timestep > 3/* && dist < 50000000*/){
        cout << "Searching, dist " << dist << endl;
        Vector move(pos - prev);
        //double r = output.find(FUEL_PORT)->second;
        //l = r;
        //if (l > move.length()){
        //    l = move.length();
        //}
        //l = -l;
        double l = move.length()*(-0.9);
        double r = move.length()*(5.0);
        bool found = false;
        double veryMinDist = 1e20;
        while (r - l > 0.1 && !found){
            VM* vm = Executer::getCloneCurrentVM();
            PortMapping in;
            in[SCENARIO_PORT] = 0;
            double m = (r+l)*0.5;
            in[VX_PORT] = (move.normalize()*m).x;
            in[VY_PORT] = (move.normalize()*m).y;

            PortMapping out = vm->step(in);
            if (out.find(SCORE_PORT)->second != 0) {
                if (out.find(SCORE_PORT)->second < 0){
                    r = m;
                    continue;
                } else {
                    assert(0);
                }
            }
            in[VX_PORT] = 0;
            in[VY_PORT] = 0;
            bool good = false;
            double prevDist = 0;
            double curDist = 0;
            Vector minPos;
            Vector minSPos;
            double minDist = 1e20;
            for (int t = 0; t < 2000000; t++){
                out = vm->step(in);
                if (out.find(SCORE_PORT)->second != 0) break;
                Vector toShipD(out.find(TARGETN_X(ship))->second, out.find(TARGETN_Y(ship))->second);
                double dist = toShipD.length();
                if (t > 3){
                    if (dist < 1000){
                        found = true;
                        good = true;
                        break;
                    }
                    if (dist > curDist && curDist < prevDist){
                        good = true;
                        break;
                    }
                    if (dist < minDist){
                        Vector pos(-out.find(EARTH_X)->second, -out.find(EARTH_Y)->second);
                        minDist = dist;
                        minPos = pos;
                        minSPos = pos+toShipD;
                    }
                }
                prevDist = curDist;
                curDist = dist;
            }
            veryMinDist = min(minDist, veryMinDist);
            if (!found && good){
                double r1 = minPos.length();
                double r2 = minSPos.length();
                if (r1 < r2){
                    l = m;
                } else {
                    r = m;
                }
            } else if (!found){
                l = m;
            }
            delete vm;
        }
        if (found){
            double m = (r+l)*0.5;
            Vector move(pos - prev);
            Vector acc(move);
            acc.normalize();
            acc *= m;
            res[VX_PORT] = acc.x;
            res[VY_PORT] = acc.y;
            searchSuccessful = true;
        } else {
            cout << "Not found\nMin distance: " << veryMinDist << endl;
            //startSearching++;
        }
        int i = 1;
    }

    prev = pos;
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

    long double r1 = orbit.maxR.length();
    long double r2 = orbit.minR.length();
    long double total_time = 2*M_PI*sqrt( pow(r1+r2,3) / (8*MU_CONST) );

    long double A,cosphi,pre_1;
    bool f = false;
    if (r > a){
        cosphi = (r-p) / (r*e);
        A = ((1-e)/(1+e));
        pre_1 = pow((1+e),2);
        f = true;
    } else {
        cosphi = (p-r) / (r*e);
        A = ((1+e)/(1-e));
        pre_1 = pow((1-e),2);
    }
    if (cosphi > 1.0) cosphi = 1.0;
    if (cosphi < -1.0) cosphi = -1.0;
    long double phi = acos(cosphi);

    long double cosphi2 = pow(cos(phi*0.5),2);
    long double cosb2 = A*cosphi2 / (1 + (A-1)*cosphi2);
    long double beta = acos(sqrt(cosb2));
//   long double tnp2 = tan(phi*0.5);
//    long double beta = tnp2 / sqrt(A);

    long double pre = 2 / (pre_1 * sqrt(A));
    long double sum1 = (-1+(1/A))*0.25*sin(2*beta);
    long double sum2 = beta * ( ((1/A) - 1)*0.5 + 1  );
    long double area = pre * (sum1 + sum2) * p * p * 0.5;

    long double total_area = M_PI * a * b; 
    //1927709481952258.5

    long double res_time = total_time * area / total_area;
    long double fixed_time = res_time;
    if (f){
        fixed_time = total_time * 0.5 - fixed_time;
    }
    //long double fixed_time = /*total_time * 0.5 - */res_time;

    Vector3D to_p(point);
    Vector3D to_apo(orbit.minR);
    Vector3D to_p_apo = Vector3D::crossProduct(to_apo, to_p);
    if ((to_p_apo.z > 0 && !orbit.clockwise) || (to_p_apo.z < 0 && orbit.clockwise)){
        fixed_time = total_time - fixed_time;
    }

    return (double)fixed_time;
}
