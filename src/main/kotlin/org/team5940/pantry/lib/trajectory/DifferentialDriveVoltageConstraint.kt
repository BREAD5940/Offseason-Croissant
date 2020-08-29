///*----------------------------------------------------------------------------*/
///* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package org.team5940.pantry.lib.trajectory
//
//import edu.wpi.first.wpilibj.geometry.Pose2d
//import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
//import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
//import org.ghrobotics.lib.mathematics.units.Meter
//import org.ghrobotics.lib.mathematics.units.SIUnit
//import org.ghrobotics.lib.mathematics.units.inMeters
//import org.ghrobotics.lib.physics.MotorCharacterization
//import kotlin.math.*
//
///**
// * A class that enforces constraints on differential drive voltage expenditure based on the motor
// * dynamics and the drive kinematics.  Ensures that the acceleration of any wheel of the robot
// * while following the trajectory is never higher than what can be achieved with the given
// * maximum voltage.
// */
//class DifferentialDriveVoltageConstraint(private val m_feedforward: MotorCharacterization<Meter>,
//                                         private val m_kinematics: DifferentialDriveKinematics,
//                                         private val m_maxVoltage: Double,
//                                         private val trackWidth: SIUnit<Meter>
//) : TrajectoryConstraint {
//
//    override fun getMaxVelocityMetersPerSecond(poseMeters: Pose2d, curvatureRadPerMeter: Double,
//                                               velocityMetersPerSecond: Double): Double = java.lang.Double.POSITIVE_INFINITY
//
//    override fun getMinMaxAccelerationMetersPerSecondSq(poseMeters: Pose2d,
//                                                        curvatureRadPerMeter: Double,
//                                                        velocityMetersPerSecond: Double): TrajectoryConstraint.MinMax {
//
//        val wheelSpeeds = m_kinematics.toWheelSpeeds(ChassisSpeeds(velocityMetersPerSecond, 0.0,
//                velocityMetersPerSecond * curvatureRadPerMeter))
//
//        val maxWheelSpeed = max(wheelSpeeds.leftMetersPerSecond,
//                wheelSpeeds.rightMetersPerSecond)
//        val minWheelSpeed = min(wheelSpeeds.leftMetersPerSecond,
//                wheelSpeeds.rightMetersPerSecond)
//
//        val maxWheelAcceleration = (m_maxVoltage - m_feedforward.kS.value.withSign(maxWheelSpeed)
//                - m_feedforward.kV.value * maxWheelSpeed) / m_feedforward.kA.value
//        val minWheelAcceleration = (-m_maxVoltage - m_feedforward.kS.value.withSign(minWheelSpeed)
//                - m_feedforward.kV.value * minWheelSpeed) / m_feedforward.kA.value
//
//        // If moving forward, max acceleration constraint corresponds to wheel on outside of turn
//        // If moving backward, max acceleration constraint corresponds to wheel on inside of turn
//        val maxChassisAcceleration = maxWheelAcceleration / (1 + (trackWidth.inMeters() * abs(curvatureRadPerMeter)
//                * sign(velocityMetersPerSecond)) / 2)
//        val minChassisAcceleration = minWheelAcceleration / (1 - (trackWidth.inMeters() * abs(curvatureRadPerMeter)
//                * sign(velocityMetersPerSecond)) / 2)
//
//        return TrajectoryConstraint.MinMax(minChassisAcceleration, maxChassisAcceleration)
//    }
//}
