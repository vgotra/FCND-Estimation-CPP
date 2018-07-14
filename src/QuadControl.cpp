#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

     /* -----------------------------------------------
	 * My lovely 99.99 working result - at least visually :) Dr.Manhattan approves this some kind of technical magic for fun :D Dr.Strange too :D
	 * In QuadControlParams.txt uncomment lines with comments " for fun"  and comment lines with comment " standard way"
	 * Try to explain this fun :) 
	 */
/*
	 // it seems that trajectory flight doesn't fully depend of this method - mostly but not fully :(
     float l = L / 1.84 * sqrtf(2);
	 float thrust_fl = collThrustCmd * 1.3;
	 float thrust_fr = momentCmd.x / l;
	 float thrust_rl = momentCmd.y / l ;
	 float thrust_rr = -(momentCmd.z / kappa);
	 float koef = 4.;

	 cmd.desiredThrustsN[0] = (thrust_fl + thrust_fr + thrust_rl + thrust_rr) / koef;
	 cmd.desiredThrustsN[1] = (thrust_fl - thrust_fr + thrust_rl - thrust_rr) / koef;
	 cmd.desiredThrustsN[2] = (thrust_fl + thrust_fr - thrust_rl - thrust_rr) / koef;
	 cmd.desiredThrustsN[3] = (thrust_fl - thrust_fr - thrust_rl + thrust_rr) / koef;
*/
	 /* ------------------------------------------ */


  float l = L * 2.f * sqrtf(2);
  float c = collThrustCmd / 4.f;
  float x = momentCmd.x / l;
  float y = momentCmd.y / l;
  float z = momentCmd.z / 4.f / kappa;

  cmd.desiredThrustsN[0] = c - z + y + x;
  cmd.desiredThrustsN[1] = c + z + y - x;
  cmd.desiredThrustsN[2] = c + z - y + x;
  cmd.desiredThrustsN[3] = c - z - y - x;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

	////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	momentCmd = kpPQR * (pqrCmd - pqr) * V3F(Ixx, Iyy, Izz);

	/////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	/* -----------------------------------------------
	* My lovely 99.99 working result - at least visually :) Dr.Manhattan approves this some kind of technical magic for fun :D Dr.Strange too :D
	* Uncomment lines with comments " for fun" in QuadControlParams.txt and comment lines with comment " standard way"
	* Try to explain this fun :) 
	*/
/*
	float c = collThrustCmd / mass;
	V3F b_command = accelCmd / c;

	float maxTiltAngleExt = 0.84;
	 
	b_command.x = -CONSTRAIN(b_command.x, -maxTiltAngleExt, maxTiltAngleExt);
	b_command.y = -CONSTRAIN(b_command.y, -maxTiltAngleExt, maxTiltAngleExt);

	float r13 = R(0, 2);
	float r23 = R(1,2);

	float kpBank2 = 13.;
	float b_x_commanded_dot = kpBank2 * (b_command.x - r13);
	float b_y_commanded_dot = kpBank2 * (b_command.y - r23);

	pqrCmd.x = (R(1, 0) * b_x_commanded_dot - R(0, 0) * b_y_commanded_dot) / R(2, 2);
	pqrCmd.y = (R(1, 1) * b_x_commanded_dot - R(0, 1) * b_y_commanded_dot) / R(2, 2);
	pqrCmd.z = 0.0;
*/
	/* ----------------------------------------------- */
	

	float c = collThrustCmd / mass;
	
	float r13 = -CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
	float r23 = -CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
	    
	if (collThrustCmd < 0)
	{
		r13 = 0;
		r23 = 0;
	}

	pqrCmd.x = (-R(1, 0) * kpBank*(R(0, 2) - r13) + R(0, 0) * kpBank*(R(1, 2) - r23)) / R(2, 2);
	pqrCmd.y = (-R(1, 1) * kpBank*(R(0, 2) - r13) + R(0, 1) * kpBank*(R(1, 2) - r23)) / R(2, 2);

	pqrCmd.z = 0.0;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	/* -----------------------------------------------
	* My lovely 99.99 working result - at least visually :) Dr.Manhattan approves this some kind of technical magic for fun :D Dr.Strange too :D
	* Uncomment lines with comments " for fun" in QuadControlParams.txt and comment lines with comment " standard way"
	* Try to explain this fun :) 
	*/
/*
	float e_z = posZCmd - posZ;
	float e_z_dot = velZCmd - velZ;	
	integratedAltitudeError += e_z * dt;

	// fun in any koef
	float kp_pos_z = 8.;
	float kp_vel_z = 13.;
	float ki_pos_z = 1.1;
	float p_dot_rate = 1.3;
	float v_dot_rate_min = 8.4;
	float v_dot_rate_max = 3.4;

	float p_dot_cmd = kp_pos_z * e_z - integratedAltitudeError + velZCmd;
	p_dot_cmd = CONSTRAIN(p_dot_cmd, -p_dot_rate, p_dot_rate);

	float v_dot_cmd = kp_vel_z * (p_dot_cmd - velZ) + e_z_dot;
	v_dot_cmd = CONSTRAIN(v_dot_cmd, -v_dot_rate_min, v_dot_rate_max);

	float c = (p_dot_cmd + v_dot_cmd + accelZCmd + ki_pos_z * integratedAltitudeError - CONST_GRAVITY) / R(2, 2);
	thrust = -c * mass;
*/
	/* ----------------------------------------------- */

	float e_z = posZCmd - posZ;
	
	integratedAltitudeError += e_z * dt;

	velZCmd += kpPosZ * e_z;
	velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

	float e_z_dot = velZCmd - velZ;	
	float result_accel = (kpVelZ * e_z_dot + KiPosZ * integratedAltitudeError + accelZCmd - 9.81f) / R(2, 2) ;
	thrust = -(result_accel * mass);

	/* -----------------------------------------------
	 * According to formullas
	 * Something from formullas - not sure why it doesn't work like expected - topic for improvements for simulator :)
	*/
	/* 
	// where is omega natural - angular velocity or rotation rate?
	auto e_z = posZCmd - posZ;
	auto e_z_dot = velZCmd - velZ;	
	auto omega_natural = e_z_dot; //velZ; 
	auto T = 1 / omega_natural; 
	// damping ratio 
	auto d_r = 0.8;
	// $K_p = \frac{1}{T^2} \cdot (1 + 2 \cdot \delta)$
	auto k_p = (1/ pow(T, 2)) * (1 + 2 * d_r);
	// $K_d = \frac{1}{T} \cdot (1 + 2 \cdot \delta)$
	auto k_d = (1/ T) * (1 + 2 * d_r);
	// $K_i = \frac{1}{T^3}$
	auto k_i = 1 / pow(T, 3);
	// or z_dot_dot_arg or current target acceleration 
	auto z_dot_dot_ff = accelZCmd; 
	auto k_p_e = k_p * e_z;
	auto k_d_e_dot = k_d * e_z_dot;
	auto k_i_int_dt = k_i * integratedAltitudeError;
	auto u1_bar = k_p_e + k_d_e_dot + k_i_int_dt + z_dot_dot_ff;
	auto u1 = mass * (u1_bar - CONST_GRAVITY);
	thrust = u1;
	*/

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	/* -----------------------------------------------
	* My lovely 99.99 working result - at least visually :) Dr.Manhattan approves this some kind of technical magic for fun :D Dr.Strange too :D
	* Uncomment lines with comments " for fun" in QuadControlParams.txt and comment lines with comment " standard way"
	* Try to explain this fun :) 
	*/
/*
 	float gKoef = 1.42; // hm - another fun - why 42 - because its answer :) (The Hitchhiker's Guide to the Galaxy)

	V3F err = posCmd - pos;
	velCmd += gKoef * err;
	V3F err_dot = velCmd - vel;

	V3F p_term = kpPosXY * err * gKoef;
	V3F d_term = kpVelXY * err_dot * gKoef;
	accelCmd = p_term + d_term + accelCmdFF;
	accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
	accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
	accelCmd.z = 0;
*/
	/* ----------------------------------------------- */

	V3F pos_err = posCmd - pos;
	V3F vel_component = kpPosXY * pos_err + velCmd;
	if (vel_component.magXY() > maxSpeedXY)
	{
	   vel_component *= maxSpeedXY / vel_component.mag();
	}

	V3F acc_component = kpVelXY * (vel_component - vel);
	accelCmd += acc_component;
	if (accelCmd.magXY() > maxAccelXY)
	{
	   accelCmd *= maxAccelXY / accelCmd.magXY();
	}

	accelCmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	float yaw_error = yawCmd - yaw;
	yaw_error = fmodf(yaw_error, 2.f*F_PI);
	if (yaw_error > F_PI)
	{
		yaw_error -= 2.f * F_PI;
	}
	else if (yaw_error < -F_PI)
	{
		yaw_error += 2.f * F_PI;
	}
	yawRateCmd = kpYaw * yaw_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
