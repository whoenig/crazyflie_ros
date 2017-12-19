#include <stdint.h>
#include <math.h>
#include "crtp.h"

//
// >>>>>>>>>>>>>>>>>>>>>>>>>> NOTE!!!!! <<<<<<<<<<<<<<<<<<<<<<<<<<<<
// The data compression routines should be kept in sync with
// datacompression.h in the Crazyflie firmware.
//

// compress a unit quaternion into 32 bits.
// assumes input quaternion is normalized. will fail if not.
static inline uint32_t quatcompress(float const q[4])
{
	// we send the values of the quaternion's smallest 3 elements.
	unsigned i_largest = 0;
	for (unsigned i = 1; i < 4; ++i) {
		if (fabsf(q[i]) > fabsf(q[i_largest])) {
			i_largest = i;
		}
	}

	// since -q represents the same rotation as q,
	// transform the quaternion so the largest element is positive.
	// this avoids having to send its sign bit.
	unsigned negate = q[i_largest] < 0;

	// 1/sqrt(2) is the largest possible value 
	// of the second-largest element in a unit quaternion.
	float const SMALL_MAX = 1.0 / sqrt(2);

	// do compression using sign bit and 9-bit precision per element.
	uint32_t comp = i_largest;
	for (unsigned i = 0; i < 4; ++i) {
		if (i != i_largest) {
			unsigned negbit = (q[i] < 0) ^ negate;
			unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / SMALL_MAX) + 0.5f;
			comp = (comp << 10) | (negbit << 9) | mag;
		}
	}

	return comp;
}

// decompress a quaternion from 32 bit compressed representation.
static inline void quatdecompress(uint32_t comp, float q[4])
{
	float const SMALL_MAX = 1.0 / sqrt(2);
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = SMALL_MAX * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}

// fixed point conversions.
// the limits on position and velocity may need to be modified depending on use case.

#define MAKE_FIXED_CONV(name, limit) \
static inline int16_t name ## _float_to_fix16(float x) \
{ \
  float normalized = x / limit; \
  if (normalized > 1.0f) normalized = 1.0f; \
  if (normalized < -1.0f) normalized = -1.0f; \
  return INT16_MAX * normalized; \
} \
static inline float name ## _fix16_to_float(int16_t x) \
{ \
  return (limit / INT16_MAX) * ((float)x); \
} \

MAKE_FIXED_CONV(position, 8.0f)
MAKE_FIXED_CONV(velocity, 20.0f)
MAKE_FIXED_CONV(accel, 20.0f)
MAKE_FIXED_CONV(attitudeRate, 20.0f)
#undef MAKE_FIXED_CONV


crtpFullStateSetpointRequest::crtpFullStateSetpointRequest(
  float x, float y, float z,
  float vx, float vy, float vz,
  float ax, float ay, float az,
  float qx, float qy, float qz, float qw,
  float rollRate, float pitchRate, float yawRate)
  : header(0x07, 0), type(6)
{
	this->x = position_float_to_fix16(x);
	this->y = position_float_to_fix16(y);
	this->z = position_float_to_fix16(z);
	this->vx = velocity_float_to_fix16(vx);
	this->vy = velocity_float_to_fix16(vy);
	this->vz = velocity_float_to_fix16(vz);
	this->ax = accel_float_to_fix16(ax);
	this->ay = accel_float_to_fix16(ay);
	this->az = accel_float_to_fix16(az);

	float q[4] = { qx, qy, qz, qw };
	this->quat = quatcompress(q);
	this->omegax = attitudeRate_float_to_fix16(rollRate);
	this->omegay = attitudeRate_float_to_fix16(pitchRate);
	this->omegaz = attitudeRate_float_to_fix16(yawRate);
}
