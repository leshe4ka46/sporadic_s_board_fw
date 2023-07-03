//=============================================================================================
// mahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================

#ifndef MAHONYAHRS_h
#define MAHONYAHRS_h
#include <math.h>

float mahony_invSqrt(float x);
void mahony_computeAngles();
void mahony_print_ptr();
void mahony_update(float invSampleFreq, float gx, float gy, float gz, float ax,
		float ay, float az, float mx, float my, float mz);
void mahony_updateIMU(float gx, float gy, float gz, float ax, float ay,
		float az);
float mahony_getRoll();
float mahony_getPitch();
float mahony_getYaw();

#endif /* MAHONYAHRS_h */
