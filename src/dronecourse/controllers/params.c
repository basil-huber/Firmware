/**
 * @file target_following_params.c
 * TargetFollowing parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


/**
* Position controller acceptance radius
*
* Radius around waypoint within which a waypoint counts as reached
*
* @min 0.0
* @max 10.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(POS_ACCEPT_RAD, 1.0f);


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
PARAM_DEFINE_FLOAT(VAR_THR_VX, 1.0f);


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
PARAM_DEFINE_FLOAT(VAR_THR_VY, 1.0f);


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
PARAM_DEFINE_FLOAT(VAR_THR_VZ, 1.0f);


/**
* TargetTracker: Kalman system noise for X position
*
* standard deviation of predicted target position in X direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_X, 0.01f);

/**
* TargetTracker: Kalman system noise for Y position
*
* standard deviation of predicted target position in Y direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_Y, 0.01f);


/**
* TargetTracker: Kalman system noise for Z position
*
* standard deviation of predicted target position in Z direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_Z, 0.01f);

/**
* TargetTracker: Kalman system noise for X velocity
*
* standard deviation of predicted target velocity in X direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_VX, 0.4f);

/**
* TargetTracker: Kalman system noise for Y velocity
*
* standard deviation of predicted target velocity in Y direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_VY, 0.4f);

/**
* TargetTracker: Kalman system noise for Z velocity
*
* confidence in predicted target velocity in Z direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_SYS_NOISE_VZ, 0.4f);

/**
* TargetTracker: Kalman measurement noise for X position
*
* standard deviation of measured target position in X direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_MEAS_NOISE_X, 0.4f);

/**
* TargetTracker: Kalman measurement noise for Y position
*
* standard deviation of measured target position in Y direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_MEAS_NOISE_Y, 0.4f);

/**
* TargetTracker: Kalman measurement noise for Z position
*
* standard deviation of measured target position in Z direction
*
* @min 0.0
* @max 500.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(KAL_MEAS_NOISE_Z, 0.4f);