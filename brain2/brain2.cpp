#include "brain2.h"

#include <vector>
#include <iostream>
#include <assert.h>

#include "vector.h"

using namespace std;

B2::B2(int sn):Brain(sn),startTime(-1) {}

PortMapping B2::step(const PortMapping& output){
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

    if (timestep == 0){
        assert(output.empty());
        res[SCENARIO_PORT] = Brain::scenarioNumber;
    } else if (timestep == 1) {
		me_r = sqrt(pow(output.find(EARTH_X)->second, 2) + pow(output.find(EARTH_Y)->second, 2));

	    Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
        Vector curTarget(output.find(TARGET_X)->second, output.find(TARGET_Y)->second);
        curTarget += curEarth;

        that_r = sqrt(pow(curTarget.x, 2) + pow(curTarget.y, 2));
		transferTime = M_PI * sqrt(pow(me_r + that_r, 3) / (8 * MU_CONST));
	} else if (timestep == 2) {
       	delta_v1 = sqrt(MU_CONST / me_r) * (sqrt(2 * that_r / (me_r + that_r)) - 1);
        
        Vector prevEarth(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
	    Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	    Vector moveDir = curEarth - prevEarth;
	    Vector tangent(curEarth.y, -curEarth.x);
	    tangent.normalize();
	    me_clockwise = ( Vector::dotProduct(moveDir, tangent) > 0.0 );
        if (!me_clockwise) {
		    tangent = -tangent;
        }
    
	    Vector prevTarget(prevInput.find(TARGET_X)->second, prevInput.find(TARGET_Y)->second);
        prevTarget += prevEarth;
	    Vector curTarget(output.find(TARGET_X)->second, output.find(TARGET_Y)->second);
        curTarget += curEarth;
	    Vector targetMoveDir = curTarget - prevTarget;
	    Vector targetTangent(curTarget.y, -curTarget.x);
	    targetTangent.normalize();
	    that_clockwise = ( Vector::dotProduct(targetMoveDir, targetTangent) > 0.0 );
        if (!that_clockwise) {
		    targetTangent = -targetTangent;
        }
        
        needTurnAround = (me_clockwise != that_clockwise);

        if (needTurnAround){
            startTime = 2;
        } 
    }
    
    if (startTime < 0 && !needTurnAround){
        Vector prevEarth(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
	    Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	    Vector prevTarget(prevInput.find(TARGET_X)->second, prevInput.find(TARGET_Y)->second);
        prevTarget += prevEarth;
	    Vector curTarget(output.find(TARGET_X)->second, output.find(TARGET_Y)->second);
        curTarget += curEarth;

        double a_me = getAngle(curEarth);
        double da_me = a_me - getAngle(prevEarth);
        
        double a_that = getAngle(curTarget);
        double da_that = a_that - getAngle(prevTarget);
        
        double rad_me = curEarth.length();
        double rad_that = curTarget.length();

        double pred_me = a_me + M_PI;
        double pred_that = a_that + transferTime * da_that;
        double dpred = abs(pred_me - pred_that);
        while (dpred >= M_PI){
            dpred -= 2*M_PI;
        }
        if (abs(dpred)*rad_that < 300){
            startTime = timestep;
        }
    }

    if (timestep == startTime){
        Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
	    Vector tangent(curEarth.y, -curEarth.x);
	    tangent.normalize();
        if (!me_clockwise) {
		    tangent = -tangent;
        }
        Vector delta = tangent * (delta_v1);
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}

	if (startTime >=0 && timestep == ceil(startTime + transferTime)) {
		assert(ceil(startTime + transferTime) > 1);

		double delta_v2 = sqrt(MU_CONST / that_r) * (1 - sqrt(2 * me_r / (me_r + that_r)));
		Vector curEarth(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);
		Vector tangent(curEarth.y, -curEarth.x);
		tangent.normalize();
        if (!me_clockwise){
			tangent = -tangent;
        }

		Vector delta = tangent * (delta_v2);
		res[VX_PORT] = delta.x;
		res[VY_PORT] = delta.y;
	}


    prevResult = res;
    prevInput = output;
	timestep++;
    return fuelOveruseFailsafe(output, res);
}

vector<pointF> B2::getShipsPositions() const{
    pointF p(-prevInput.find(EARTH_X)->second, -prevInput.find(EARTH_Y)->second);
    vector<pointF> res;
    res.push_back(p);
    pointF target( p.first + prevInput.find(TARGET_X)->second, p.second + prevInput.find(TARGET_Y)->second   );
    res.push_back(target);
    return res;
}

int B2::getShipsNumber() const{
    return 2;
}

double B2::getAngle(const Vector& v) const{
    Vector norm(v);
    norm.normalize();
    double res = acos(norm.x);
    if (norm.y < 0){
        res = -res;
    }
    return res;
}
