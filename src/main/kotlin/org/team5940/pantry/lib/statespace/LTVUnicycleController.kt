package org.team5940.pantry.lib.statespace

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import kotlin.math.absoluteValue
import kotlin.math.sqrt
import kotlin.math.withSign
import org.ejml.simple.SimpleMatrix
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meters

/**
 * A cascaded Linear time-varying unicycle controller. See Theorem 8.7.2
 * from https://github.com/calcmogul/controls-engineering-in-frc
 */
class LTVUnicycleController(
    private val kX: Double,
    private val kY_0: Double,
    private val kY_1: Double,
    private val kTheta: Double
) {

    /**
     * Calculate the desired state
     */
//    fun calculateState(iterator: TrajectoryIterator<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>, robotPose: Pose2d): ChassisSpeeds {
//        val referenceState = iterator.currentState.state
//
//        // Get reference linear and angular velocities
//        val vd = referenceState.velocity.value
//        val wd = vd * referenceState.state.curvature
//
//        val newState = calculate(robotPose, referenceState.state.pose, vd, wd)
//
//        return ChassisSpeeds(newState.linear.meter.velocity, 0.0, newState.angular.radian.velocity)
//
//    }

    private var poseError = Pose2d()

    fun calculate(currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMetersPerSec: Double, angularVelocityRefRadiansPerSecond: Double): ChassisSpeeds {
        this.poseError = poseRef.relativeTo(currentPose)
        val eX = this.poseError.translation.x
        val eY = this.poseError.translation.y
        val eTheta = this.poseError.rotation.radians

        val error = SimpleMatrix(3, 1, false, doubleArrayOf(
                eX, eY, eTheta
        ))

        val u = K(linearVelocityRefMetersPerSec).mult(error)

        val toRet = ChassisSpeeds(u[0] + linearVelocityRefMetersPerSec, 0.0,
                u[1] + angularVelocityRefRadiansPerSecond)

        println("Commanding linear ${toRet.vxMetersPerSecond.meters.feet} angluar ${toRet.omegaRadiansPerSecond.radians.degree}")

        return toRet
    }

    fun K(velocity: Double) = SimpleMatrix(2, 3, true, doubleArrayOf(
            kX, 0.0, 0.0,
            0.0, kY(velocity).withSign(velocity), kTheta * sqrt(velocity.absoluteValue)
    ))

    fun kY(velocity: Double): Double {
        return kY_0 + (kY_1 - kY_0) * sqrt(velocity.absoluteValue)
    }

    fun calculate(currentPose: Pose2d, desiredState: Trajectory.State) =
            calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond,
                    desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter)
}
