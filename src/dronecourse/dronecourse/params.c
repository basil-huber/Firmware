/**
 * @file target_following_params.c
 * TargetFollowing parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

/**
* TargetFollower velocity weight
*
* Selects weight of velocity error
*
* @min 0.0
* @max 2.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(FOL_VEL, 1.0f);


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