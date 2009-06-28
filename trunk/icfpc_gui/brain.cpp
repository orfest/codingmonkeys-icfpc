#include "brain.h"

#include <exception>
#include <iostream>
#include <assert.h>
#include <algorithm>

#include "brain1.h"
#include "brain2.h"
#include "brain2_2.h"
#include "brain3.h"
#include "brain3_3.h"
#include "brain4.h"
#include "vector.h"

#include "vm.h"
#include "executer.h"

using namespace std;

PortMapping Brain::prevInput = PortMapping();
PortMapping Brain::prevResult = PortMapping();
const double Brain::fuelEps = 1e-3;

Brain::Brain(int sn, VM* vm_):scenarioNumber(sn),timestep(0),vm(vm_){}

static double eps = 1;
static double epsConsiderCircle = 1e4;

Brain* Brain::getBrain(int problem, int scenarioNumber, VM* vm){
    if (problem == 0) {
        return new B1(scenarioNumber, vm);
	} else if (problem == 1) {
        return new B2_2(scenarioNumber, vm);
    } else if (problem == 2) {
        //return new B3_3(scenarioNumber, vm);
		return new B3(scenarioNumber, vm);
    } else if (problem == 3) {
        return new B4(scenarioNumber, vm);
    } else {
        throw new std::exception("Unknown problem type");
    }
}

bool Brain::finished(const PortMapping& output) const{
    data_t score = output.find(SCORE_PORT)->second;
    if (score == 0) return false;
    std::cout << 
        "Scenario "     << scenarioNumber << " completed!\n" << 
        "Final score: " << score << "\n" << 
        "Time: "        << timestep << "\n" << 
        "Fuel remaining: " << output.find(FUEL_PORT)->second << std::endl;
    return true;
}

PortMapping & Brain::fuelOveruseFailsafe(const PortMapping & sensors, PortMapping & actuators) {
	if (sensors.empty())
		return actuators;

	double fuelAvailable = sensors.find(FUEL_PORT)->second;
	Vector delta(actuators.find(VX_PORT)->second, actuators.find(VY_PORT)->second);
	if (fuelAvailable - fuelEps < delta.length()) {
		delta.normalize();
		delta *= (fuelAvailable > fuelEps ? fuelAvailable - fuelEps : 0.0);
		actuators[VX_PORT] = delta.x;
		actuators[VY_PORT] = delta.y;
	}
	return actuators;
}


PortMapping Brain::step(const PortMapping& output) {
	// hack for Brain4 stub !!!
	// to use Brain3 (B3) with less bugs add similar hack
    if (scenarioNumber / 1000 == 3 || scenarioNumber / 1000 == 4) {
		return _step(output);
    }

	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

	if (timestep == 0){
		assert(output.empty());
		res[SCENARIO_PORT] = Brain::scenarioNumber;
		state = WAITING;

        simulateAndGetOrbits();

	}else if (timestep > 1){
		do {
			if (state == COMPLETE)
				state = WAITING;
			if (state == WAITING){
				if (operation_list.empty()){
					_step(output);
				}
				if (!operation_list.empty()){
					Operation* oper = operation_list.front();
					state = RUNNING;
					PortMapping tmp = oper->step(output);
					res[VX_PORT] += tmp[VX_PORT];
					res[VY_PORT] += tmp[VY_PORT];
					if (oper->state == COMPLETE){
						state = COMPLETE;
						operation_list.pop();
					}
				}
			} else if (state == RUNNING) {
				Operation* oper = operation_list.front();
				res = oper->step(output);
				if (oper->state == COMPLETE){
					state = COMPLETE;
					operation_list.pop();
				}
			}
		} while (state == COMPLETE);
	}

	prevResult = res;
	prevInput = output;

	timestep++;
	return fuelOveruseFailsafe(output, res);
}

void Brain::simulateAndGetOrbits(){
	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;

    int numShips = this->getShipsNumber();
    orbits.resize(numShips);
    vector<double> maxDist(numShips, -1e20);
    vector<double> minDist(numShips, 1e20);
    vector<int> maxDistTime(numShips, -1);
    vector<int> minDistTime(numShips, -1);
    vector<int> done(numShips, 0);
    vector<double> startPolarAngle(numShips);
    vector<Vector> prev(numShips);

    int toexamine = numShips;
    for (int t = 0; toexamine > 0 && t < 3000000; t++){
        if (t == 0){
            res[SCENARIO_PORT] = scenarioNumber;
        }
        PortMapping output = vm->step(res);
        vector<pointF> shipsPositions = this->getShipsPositions(output);
        assert(shipsPositions.size() == orbits.size());
        for (int i = 0; i < numShips; i++){
            if (done[i]) continue;
            Vector pos(shipsPositions[i].first, shipsPositions[i].second);
            if (t == 0){
                startPolarAngle[i] = getPolarAngle(pos);
            }
            double dist = pos.length();
            if (dist > maxDist[i] + eps){
                maxDist[i] = dist;
                maxDistTime[i] = t;
                orbits[i].maxR = pos;
            }
            if (dist < minDist[i] - eps){
                minDist[i] = dist;
                minDistTime[i] = t;
                orbits[i].minR = pos;
            }
            if (t > 1000){
                double curPolarAngle = getPolarAngle(pos);
                double diffPolarAngle = curPolarAngle - startPolarAngle[i];
                while (diffPolarAngle < -M_PI) diffPolarAngle += 2*M_PI;
                while (diffPolarAngle > M_PI) diffPolarAngle -= 2*M_PI;
                if (abs(diffPolarAngle) < 0.002){
                    done[i] = 1;
                    toexamine--;
                    orbits[i].clockwise = isClockwise(pos, prev[i]);
                }
            }
            prev[i] = pos;
        }
        if (t == 0){
            res[SCENARIO_PORT] = 0;
        }
    }
    int i = 1;
}

bool Brain::isClockwise(const Vector& newPosition, const Vector& prevPosition) const{
    Vector move(newPosition-prevPosition);
    
    Vector tangent(newPosition.y, -newPosition.x);      // turned 90 clockwise
	bool clockwise = ( Vector::dotProduct(move, tangent) > 0.0 );
    return clockwise;
}

// returns value in range [-pi, pi]
double Brain::getPolarAngle(const Vector& v) const {
    Vector norm(v);
    norm.normalize();
    double res = acos(norm.x);
    if (norm.y < 0){
        res = -res;
    }
    return res;
}

int Brain::estimateTimeToPoint(const Vector& point) const{
    static const double LIMIT = 2000;
    VM* vm = Executer::getCloneCurrentVM();
 	PortMapping res;
	res[SCENARIO_PORT] = 0;
	res[VX_PORT] = 0;
	res[VY_PORT] = 0;
    PortMapping output;
    for (int t = 0; t < 3000000; t++){
        output = vm->step(res);
        Vector current(-output.find(EARTH_X)->second, -output.find(EARTH_Y)->second);       //!!
        current -= point;
        if (current.length() < LIMIT){
            delete vm;
            return t+1;
        }
    }
    delete vm;
    return -1;
}

// will return aphelion and perihelion positions for arbitrary ellipse orbit
// given orbit point and velocity at that point
void Brain::estimateOrbit(const Vector & velocity, const Vector & position, 
										Vector & aphelionPos, Vector & perihelionPos) const {
	assert(velocity.length() > 1e-3);
	assert(position.length() > 1e-3);

	struct Vector3D {
		double x, y, z;
		Vector3D(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
		Vector3D(const Vector3D & vec) : x(vec.x), y(vec.y), z(vec.z) {}
		Vector3D(const Vector & vec) : x(vec.x), y(vec.y), z(0.0) {}
		static Vector3D crossProduct(const Vector3D & a, const Vector3D & b) {
			return Vector3D(a.y * b.z - a.z * b.y, 
							a.z * b.x - a.x * b.z, 
							a.x * b.y - a.y * b.x);
		}
	};

	Vector3D r(position);
	Vector3D v(velocity);
	Vector3D e = Vector3D::crossProduct(v, Vector3D::crossProduct(r, v));
	assert(abs(e.z) < 1e-6);

	Vector eccent(e.x, e.y);
	eccent = eccent / MU_CONST - Vector(position).normalize();
	Vector aphelionDir(eccent);
	aphelionDir.normalize();
	double eccentMod = eccent.length();

	double energy = velocity.sqLength() / 2 - MU_CONST / position.length();
	double sMjAxis = - MU_CONST / (2 * energy);
#if 0	
	// those values may be useful
	double l = Vector::crossProduct(position, velocity);
	double alpha = l * l / MU_CONST;
	double sMnAxis = sqrt(alpha * sMjAxis);
	double sqEccent = 1 - pow(sMnAxis / sMjAxis, 2);
	double eccentricity = sqrt(sqEccent);
	assert(abs(eccentricity - eccentMod) < 1e-6);
#endif

	aphelionPos = aphelionDir * ((1 - eccentMod) * sMjAxis);
	perihelionPos = - aphelionDir * ((1 + eccentMod) * sMjAxis);
}

Vector Brain::getVectorFromPolarAngle(double angle, double scale) const {
	return Vector(cos(angle), sin(angle)) * scale;
}

// returns (absolutely minimum) signed difference in phase between two vectors
// if positive values mean that shortest way to get from 'a' to 'b' is clockwise
double Brain::getPhaseDifference(const Vector & a, const Vector & b) const {
	double angA = getPolarAngle(a);
	double angB = getPolarAngle(b);
	if (angA - angB > M_PI)
		return angA - (angB + 2*M_PI);
	else if (angA - angB >= 0)
		return angA - angB;
	else if (angA - angB >= -M_PI)
		return angA + 2*M_PI - angB;
	else
		return angA - angB;
}

// Whether it is OK with given precision 'epsilon' to start orbit transfer when 
// we are in point 'vec' and should start transfer in point 'aphOrPer', which is
// opposite to point 'perOrAph' on current orbit. This function is circle orbits aware.
// Determinig circle with precision of 'epsConsiderCircle' (in-module).
bool Brain::isPhaseWithinEpsCircleAware(const Vector & vec, const Vector & aphOrPer, 
								  const Vector & perOrAph, double epsilon) const {
	if (abs(aphOrPer.length() - perOrAph.length()) < epsConsiderCircle) { // any phase is good for circle
		return true;
	}
	return abs(getPhaseDifference(vec, aphOrPer)) < epsilon;
}
