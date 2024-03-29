/* ***********************************************************
 * This file was automatically generated on 2012-12-20.      *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generator git on tinkerforge.com                   *
 *************************************************************/

#ifndef BRICK_IMU_H
#define BRICK_IMU_H

#include "ip_connection.h"

/**
 * \defgroup BrickIMU IMU Brick
 */

/**
 * \ingroup BrickIMU
 *
 * Device for sensing acceleration, magnetic field and angular velocity
 */
typedef Device IMU;

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_acceleration_period}. The parameters are the acceleration
 * for the x, y and z axis.
 */
#define IMU_CALLBACK_ACCELERATION 31

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_magnetic_field_period}. The parameters are the magnetic field
 * for the x, y and z axis.
 */
#define IMU_CALLBACK_MAGNETIC_FIELD 32

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_angular_velocity_period}. The parameters are the angular velocity
 * for the x, y and z axis.
 */
#define IMU_CALLBACK_ANGULAR_VELOCITY 33

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_all_data_period}. The parameters are the acceleration,
 * the magnetic field and the angular velocity for the x, y and z axis as
 * well as the temperature of the IMU Brick.
 */
#define IMU_CALLBACK_ALL_DATA 34

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_orientation_period}. The parameters are the orientation
 * (roll, pitch and yaw) of the IMU Brick in Euler angles. See
 * {@link imu_get_orientation} for details.
 */
#define IMU_CALLBACK_ORIENTATION 35

/**
 * \ingroup BrickIMU
 *
 * This callback is triggered periodically with the period that is set by
 * {@link imu_set_quaternion_period}. The parameters are the orientation
 * (x, y, z, w) of the IMU Brick in quaternions. See {@link imu_get_quaternion}
 * for details.
 */
#define IMU_CALLBACK_QUATERNION 36

/**
 * \ingroup BrickIMU
 *
 * Creates an object with the unique device ID \c uid. This object can then be
 * added to the IP connection.
 */
void imu_create(IMU *imu, const char *uid);

/**
 * \ingroup BrickIMU
 *
 * Returns the calibrated acceleration from the accelerometer for the 
 * x, y and z axis in mG (G/1000, 1G = 9.80605m/s²).
 * 
 * If you want to get the acceleration periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_ACCELERATION} and set the period with 
 * {@link imu_set_acceleration_period}.
 */
int imu_get_acceleration(IMU *imu, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z);

/**
 * \ingroup BrickIMU
 *
 * Returns the calibrated magnetic field from the magnetometer for the 
 * x, y and z axis in mG (Milligauss or Nanotesla).
 * 
 * If you want to get the magnetic field periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_MAGNETIC_FIELD} and set the period with 
 * {@link imu_set_magnetic_field_period}.
 */
int imu_get_magnetic_field(IMU *imu, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z);

/**
 * \ingroup BrickIMU
 *
 * Returns the calibrated angular velocity from the gyroscope for the 
 * x, y and z axis in °/17.5s (you have to divide by 17.5 to
 * get the value in °/s).
 * 
 * If you want to get the angular velocity periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_ANGULAR_VELOCITY} and set the period with 
 * {@link imu_set_angular_velocity_period}.
 */
int imu_get_angular_velocity(IMU *imu, int16_t *ret_x, int16_t *ret_y, int16_t *ret_z);

/**
 * \ingroup BrickIMU
 *
 * Returns the data from {@link imu_get_acceleration}, {@link imu_get_magnetic_field} 
 * and {@link imu_get_angular_velocity} as well as the temperature of the IMU Brick.
 * 
 * The temperature is given in °C/100.
 * 
 * If you want to get the data periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_ALL_DATA} and set the period with 
 * {@link imu_set_all_data_period}.
 */
int imu_get_all_data(IMU *imu, int16_t *ret_acc_x, int16_t *ret_acc_y, int16_t *ret_acc_z, int16_t *ret_mag_x, int16_t *ret_mag_y, int16_t *ret_mag_z, int16_t *ret_ang_x, int16_t *ret_ang_y, int16_t *ret_ang_z, int16_t *ret_temperature);

/**
 * \ingroup BrickIMU
 *
 * Returns the current orientation (roll, pitch, yaw) of the IMU Brick as Euler
 * angles in one-hundredth degree. Note that Euler angles always experience a
 * `gimbal lock <http://en.wikipedia.org/wiki/Gimbal_lock>`__.
 * 
 * We recommend that you use quaternions instead.
 * 
 * The order to sequence in which the orientation values should be applied is 
 * roll, yaw, pitch. 
 * 
 * If you want to get the orientation periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_ORIENTATION} and set the period with 
 * {@link imu_set_orientation_period}.
 */
int imu_get_orientation(IMU *imu, int16_t *ret_roll, int16_t *ret_pitch, int16_t *ret_yaw);

/**
 * \ingroup BrickIMU
 *
 * Returns the current orientation (x, y, z, w) of the IMU as 
 * `quaternions <http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>`__.
 * 
 * You can go from quaternions to Euler angles with the following formula::
 * 
 *  roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
 *  pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
 *  yaw   =  asin(2*x*y + 2*z*w)
 * 
 * This process is not reversible, because of the 
 * `gimbal lock <http://en.wikipedia.org/wiki/Gimbal_lock>`__.
 * 
 * Converting the quaternions to an OpenGL transformation matrix is
 * possible with the following formula::
 * 
 *  matrix = [[1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y), 0],
 *            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x), 0],
 *            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y), 0],
 *            [                0,                 0,                 0, 1]]
 * 
 * If you want to get the quaternions periodically, it is recommended 
 * to use the callback {@link IMU_CALLBACK_QUATERNION} and set the period with 
 * {@link imu_set_quaternion_period}.
 */
int imu_get_quaternion(IMU *imu, float *ret_x, float *ret_y, float *ret_z, float *ret_w);

/**
 * \ingroup BrickIMU
 *
 * Returns the temperature of the IMU Brick. The temperature is given in 
 * °C/100.
 */
int imu_get_imu_temperature(IMU *imu, int16_t *ret_temperature);

/**
 * \ingroup BrickIMU
 *
 * Turns the orientation and direction LEDs of the IMU Brick on.
 */
int imu_leds_on(IMU *imu);

/**
 * \ingroup BrickIMU
 *
 * Turns the orientation and direction LEDs of the IMU Brick off.
 */
int imu_leds_off(IMU *imu);

/**
 * \ingroup BrickIMU
 *
 * Returns *true* if the orientation and direction LEDs of the IMU Brick
 * are on, *false* otherwise.
 */
int imu_are_leds_on(IMU *imu, bool *ret_leds);

/**
 * \ingroup BrickIMU
 *
 * Not implemented yet.
 */
int imu_set_acceleration_range(IMU *imu, uint8_t range);

/**
 * \ingroup BrickIMU
 *
 * Not implemented yet.
 */
int imu_get_acceleration_range(IMU *imu, uint8_t *ret_range);

/**
 * \ingroup BrickIMU
 *
 * Not implemented yet.
 */
int imu_set_magnetometer_range(IMU *imu, uint8_t range);

/**
 * \ingroup BrickIMU
 *
 * Not implemented yet.
 */
int imu_get_magnetometer_range(IMU *imu, uint8_t *ret_range);

/**
 * \ingroup BrickIMU
 *
 * Sets the convergence speed of the IMU Brick in °/s. The convergence speed 
 * determines how the different sensor measurements are fused.
 * 
 * If the orientation of the IMU Brick is off by 10° and the convergence speed is 
 * set to 20°/s, it will take 0.5s until the orientation is corrected. However,
 * if the correct orientation is reached and the convergence speed is too high,
 * the orientation will fluctuate with the fluctuations of the accelerometer and
 * the magnetometer.
 * 
 * If you set the convergence speed to 0, practically only the gyroscope is used
 * to calculate the orientation. This gives very smooth movements, but errors of the
 * gyroscope will not be corrected. If you set the convergence speed to something
 * above 500, practically only the magnetometer and the accelerometer are used to
 * calculate the orientation. In this case the movements are abrupt and the values
 * will fluctuate, but there won't be any errors that accumulate over time.
 * 
 * In an application with high angular velocities, we recommend a high convergence
 * speed, so the errors of the gyroscope can be corrected fast. In applications with
 * only slow movements we recommend a low convergence speed. You can change the
 * convergence speed on the fly. So it is possible (and recommended) to increase 
 * the convergence speed before an abrupt movement and decrease it afterwards 
 * again.
 * 
 * You might want to play around with the convergence speed in the Brick Viewer to
 * get a feeling for a good value for your application.
 * 
 * The default value is 30.
 */
int imu_set_convergence_speed(IMU *imu, uint16_t speed);

/**
 * \ingroup BrickIMU
 *
 * Returns the convergence speed as set by {@link imu_set_convergence_speed}.
 */
int imu_get_convergence_speed(IMU *imu, uint16_t *ret_speed);

/**
 * \ingroup BrickIMU
 *
 * There are several different types that can be calibrated:
 * 
 * \verbatim
 *  "Type", "Description",        "Values"
 * 
 *  "0",    "Accelerometer Gain", "[mul x, mul y, mul z, div x, div y, div z, 0, 0, 0, 0]"
 *  "1",    "Accelerometer Bias", "[bias x, bias y, bias z, 0, 0, 0, 0, 0, 0, 0]"
 *  "2",    "Magnetometer Gain",  "[mul x, mul y, mul z, div x, div y, div z, 0, 0, 0, 0]"
 *  "3",    "Magnetometer Bias",  "[bias x, bias y, bias z, 0, 0, 0, 0, 0, 0, 0]"
 *  "4",    "Gyroscope Gain",     "[mul x, mul y, mul z, div x, div y, div z, 0, 0, 0, 0]"
 *  "5",    "Gyroscope Bias",     "[bias xl, bias yl, bias zl, temp l, bias xh, bias yh, bias zh, temp h, 0, 0]"
 * \endverbatim
 * 
 * The calibration via gain and bias is done with the following formula::
 * 
 *  new_value = (bias + orig_value) * gain_mul / gain_div
 * 
 * If you really want to write your own calibration software, please keep
 * in mind that you first have to undo the old calibration (set bias to 0 and
 * gain to 1/1) and that you have to average over several thousand values
 * to obtain a usable result in the end.
 * 
 * The gyroscope bias is highly dependent on the temperature, so you have to
 * calibrate the bias two times with different temperatures. The values xl, yl, zl 
 * and temp l are the bias for x, y, z and the corresponding temperature for a 
 * low temperature. The values xh, yh, zh and temp h are the same for a high 
 * temperatures. The temperature difference should be at least 5°C. If you have 
 * a temperature where the IMU Brick is mostly used, you should use this 
 * temperature for one of the sampling points.
 * 
 * \note
 *  We highly recommend that you use the Brick Viewer to calibrate your
 *  IMU Brick.
 */
int imu_set_calibration(IMU *imu, uint8_t typ, int16_t data[10]);

/**
 * \ingroup BrickIMU
 *
 * Returns the calibration for a given type as set by {@link imu_set_calibration}.
 */
int imu_get_calibration(IMU *imu, uint8_t typ, int16_t ret_data[10]);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_ACCELERATION} callback is triggered
 * periodically. A value of 0 turns the callback off.
 * 
 * The default value is 0.
 */
int imu_set_acceleration_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_acceleration_period}.
 */
int imu_get_acceleration_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_MAGNETIC_FIELD} callback is triggered
 * periodically. A value of 0 turns the callback off.
 */
int imu_set_magnetic_field_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_magnetic_field_period}.
 */
int imu_get_magnetic_field_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_ANGULAR_VELOCITY} callback is triggered
 * periodically. A value of 0 turns the callback off.
 */
int imu_set_angular_velocity_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_angular_velocity_period}.
 */
int imu_get_angular_velocity_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_ALL_DATA} callback is triggered
 * periodically. A value of 0 turns the callback off.
 */
int imu_set_all_data_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_all_data_period}.
 */
int imu_get_all_data_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_ORIENTATION} callback is triggered
 * periodically. A value of 0 turns the callback off.
 */
int imu_set_orientation_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_orientation_period}.
 */
int imu_get_orientation_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Sets the period in ms with which the {@link IMU_CALLBACK_QUATERNION} callback is triggered
 * periodically. A value of 0 turns the callback off.
 */
int imu_set_quaternion_period(IMU *imu, uint32_t period);

/**
 * \ingroup BrickIMU
 *
 * Returns the period as set by {@link imu_set_quaternion_period}.
 */
int imu_get_quaternion_period(IMU *imu, uint32_t *ret_period);

/**
 * \ingroup BrickIMU
 *
 * Calling this function will reset the Brick. Calling this function
 * on a Brick inside of a stack will reset the whole stack.
 * 
 * After a reset you have to create new device objects,
 * calling functions on the existing ones will result in
 * undefined behavior!
 * 
 * .. versionadded:: 1.0.7~(Firmware)
 */
int imu_reset(IMU *imu);

/**
 * \ingroup BrickIMU
 *
 * Returns the temperature in °C/10 as measured inside the microcontroller. The
 * value returned is not the ambient temperature!
 * 
 * The temperature is only proportional to the real temperature and it has an
 * accuracy of +-15%. Practically it is only useful as an indicator for
 * temperature changes.
 * 
 * .. versionadded:: 1.0.7~(Firmware)
 */
int imu_get_chip_temperature(IMU *imu, int16_t *ret_temperature);

/**
 * \ingroup BrickIMU
 *
 * Returns the name (including the hardware version), the firmware version
 * and the binding version of the device. The firmware and binding versions are
 * given in arrays of size 3 with the syntax [major, minor, revision].
 */
int imu_get_version(IMU *imu, char ret_name[40], uint8_t ret_firmware_version[3], uint8_t ret_binding_version[3]);

/**
 * \ingroup BrickIMU
 *
 * Registers a callback with ID \c id to the function \c callback.
 */
void imu_register_callback(IMU *imu, uint8_t id, void *callback);

#endif
