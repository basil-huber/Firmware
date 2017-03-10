/**
 * @file target_following_params.c
 * TargetFollowing parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


/**
* TargetFollower position weight
*
* Selects weight of position error
*
* @min 0.0
* @max 2.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(FOL_POS, 0.1f);


/**
* TargetFollower threshold variance for X position estimate
*
* Threshold for confidence in target position estimate in X direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_X, 70.0f);


/**
* TargetFollower threshold variance for Y position estimate
*
* Threshold for confidence in target position estimate in Y direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_Y, 70.0f);


/**
* TargetFollower threshold variance for Z position estimate
*
* Threshold for confidence in target position estimate in Z direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_Z, 70.0f);


/**
* TargetFollower threshold variance for X velocity estimate
*
* Threshold for confidence in target velocity estimate in X direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_VX, 2.0f);


/**
* TargetFollower threshold variance for Y velocity estimate
*
* Threshold for confidence in target velocity estimate in Y direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_VY, 2.0f);


/**
* TargetFollower threshold variance for Z velocity estimate
*
* Threshold for confidence in target velocity estimate in Z direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(VAR_THR_VZ, 2.0f);