/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team5940.pantry.lib.trajectory

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint

/**
 * A class that enforces constraints on the differential drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for both sides of the drivetrain stay below a certain
 * limit.
 */
class DifferentialDriveKinematicsConstraint
/**
 * Constructs a differential drive dynamics constraint.
 *
 * @param kinematics A kinematics component describing the drive geometry.
 * @param maxSpeedMetersPerSecond The max speed that a side of the robot can travel at.
 */
(private val m_kinematics: DifferentialDriveKinematics,
 private val m_maxSpeedMetersPerSecond: Double) : TrajectoryConstraint {


    /**
     * Returns the max velocity given the current pose and curvature.
     *
     * @param poseMeters              The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter    The curvature at the current point in the trajectory.
     * @param velocityMetersPerSecond The velocity at the current point in the trajectory before
     * constraints are applied.
     * @return The absolute maximum velocity.
     */
    override fun getMaxVelocityMetersPerSecond(poseMeters: Pose2d, curvatureRadPerMeter: Double,
                                               velocityMetersPerSecond: Double): Double {
        // Create an object to represent the current chassis speeds.
        val chassisSpeeds = ChassisSpeeds(velocityMetersPerSecond,
                0.0, velocityMetersPerSecond * curvatureRadPerMeter)

        // Get the wheel speeds and normalize them to within the max velocity.
        val wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds)
        wheelSpeeds.normalize(m_maxSpeedMetersPerSecond)

        // Return the new linear chassis speed.
        return m_kinematics.toChassisSpeeds(wheelSpeeds).vxMetersPerSecond
    }

    /**
     * Returns the minimum and maximum allowable acceleration for the trajectory
     * given pose, curvature, and speed.
     *
     * @param poseMeters              The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter    The curvature at the current point in the trajectory.
     * @param velocityMetersPerSecond The speed at the current point in the trajectory.
     * @return The min and max acceleration bounds.
     */
    override fun getMinMaxAccelerationMetersPerSecondSq(poseMeters: Pose2d,
                                                        curvatureRadPerMeter: Double,
                                                        velocityMetersPerSecond: Double): TrajectoryConstraint.MinMax {
        return TrajectoryConstraint.MinMax()
    }
}
