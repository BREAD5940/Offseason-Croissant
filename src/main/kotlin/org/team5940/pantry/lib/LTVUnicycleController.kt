package org.team5940.pantry.lib

import com.team254.lib.physics.DifferentialDrive
import frc.robot.Constants
import org.ejml.simple.SimpleMatrix
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryIterator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.*

/**
 * A cascaded Linear time-varying unicycle controller. See Theorem 8.7.2
 * from https://github.com/calcmogul/controls-engineering-in-frc
 */
class LTVUnicycleController(
        private val kX: Double,
        private val kY_0: Double,
        private val kY_1: Double,
        private val kTheta: Double,
        private val diffDrive: DifferentialDrive, private val maxVoltage: SIUnit<Volt>
): TrajectoryTracker()  {

    /**
     * Calculate the desired state
     */
    override fun calculateState(iterator: TrajectoryIterator<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>, robotPose: Pose2d): TrajectoryTrackerVelocityOutput {
        val referenceState = iterator.currentState.state

        // Get reference linear and angular velocities
        val vd = referenceState.velocity.value
        val wd = vd * referenceState.state.curvature

        val newState = calculate(robotPose, referenceState.state.pose, vd, wd)

        // angular velocity / linVelocity = curvature
        val curvature = newState.angular / newState.linear
        val maxSpeed = diffDrive.getMaxAbsVelocity(curvature, maxVoltage.value)

        return TrajectoryTrackerVelocityOutput(newState.linear.coerceIn(-maxSpeed, maxSpeed).meter.velocity, newState.angular.radian.velocity)
    }

    private var poseError = Pose2d()
    var poseTolerance = Pose2d()

    fun calculate(currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMetersPerSec: Double, angularVelocityRefRadiansPerSecond: Double): DifferentialDrive.ChassisState {
        this.poseError = poseRef.inFrameOfReferenceOf(currentPose)
        val eX = this.poseError.translation.x
        val eY = this.poseError.translation.y
        val eTheta = this.poseError.rotation.radian
        val heading = currentPose.rotation.radian

//        // Rotate error into robot reference frame
//        val rotationMatrix = SimpleMatrix(3, 3, true, doubleArrayOf(
//            cos(heading), sin(heading), 0.0,
//            -sin(heading), cos(heading), 0.0,
//            0.0, 0.0, 1.0
//        ))

        val error = SimpleMatrix(3, 1, false, doubleArrayOf(
                eX.meter, eY.meter, eTheta
        ))

        val rotatedError = error// rotationMatrix.mult(error)


        val u = K(linearVelocityRefMetersPerSec).mult(rotatedError)

        val string = K(linearVelocityRefMetersPerSec).toString()

        val u0 = u[0]
        val u1 = u[1]

        val x = rotatedError[0]
        val y = rotatedError[1]
        val t = rotatedError[2]

        val toRet = DifferentialDrive.ChassisState(u[0] + linearVelocityRefMetersPerSec,
                u[1] + angularVelocityRefRadiansPerSecond)

        println("Commanding linear ${toRet.linear.meter.feet} angluar ${toRet.angular.radian.degree}")

        return toRet
    }

    fun K(velocity: Double) = SimpleMatrix(2, 3, true, doubleArrayOf(
            kX, 0.0, 0.0,
            0.0, kY(velocity).withSign(velocity), kTheta * sqrt(velocity.absoluteValue)
    ))

    fun kY(velocity: Double): Double {
        return kY_0 + (kY_1 - kY_0) * sqrt(velocity.absoluteValue)
    }

//    fun calculate(currentPose: Pose2d, desiredState: Trajectory.State) =
//            calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond,
//                    desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter)

}

/**
 * A unicycle controller with gains calculated at q = [0.1, 0.1, 10deg] and r = [3.96meters/sec, 180deg/second]
 */
val defaultLTVUnicycleController get() = LTVUnicycleController(
        11.195525843132865, 19.410289939191696, 18.523330615355434, 6.33841334186717, Constants.DriveConstants.kHighGearDifferentialDrive, 10.volt
)