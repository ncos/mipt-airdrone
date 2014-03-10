
#include "position.h"

#include <cmath>
#define PI 3.14159265

Position::Position ()
{
	this->normal_to_floor(0) = 0;  // Component to the front
	this->normal_to_floor(1) = 1;  // Axis up
	this->normal_to_floor(2) = 0;  // Axis right (left)
}

void Position::renew (float &A, float &B, float &C, float &D, uint64 num_of_points)
{
	if (num_of_points < 200) {this->is_Nan = true; return;}
	this->is_Nan = false;

	this->normal_to_floor(0) = A;  // Component to the front
	this->normal_to_floor(1) = B;  // Axis up
	this->normal_to_floor(2) = C;  // Axis right (left)

	// (0, 1, 0) - vector normal to floor
	double roll_cos  = B/sqrt(B*B + C*C);
	double pitch_cos = B/sqrt(B*B + A*A);

	this->rotation.pitch = (PI-acos(pitch_cos)) * 180.0/PI;
	this->rotation.roll  = (PI-acos(roll_cos )) * 180.0/PI;

	this->distance_to_floor = D;
}
