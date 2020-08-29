package org.team5940.pantry.lib.statespace

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import kotlin.math.absoluteValue
import kotlin.math.sqrt
import kotlin.math.withSign
import org.ejml.simple.SimpleMatrix
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.mathematics.units.meters

data class LTVDiffDriveGains(
    internal val kX: Double,
    internal val kY_0: Double,
    internal val kvPlus_0: Double,
    internal val kvMinus_0: Double,
    internal val kY_1: Double,
    internal val kTheta_1: Double,
    internal val kvPlus_1: Double,
    internal val kinematics: DifferentialDriveKinematics
)

/**
 * A Linear time-varying differential drive controller. See Theorem 8.6.2
 * from https://github.com/calcmogul/controls-engineering-in-frc
 */
@Suppress("PrivatePropertyName", "FunctionName")
class LTVDiffDriveController(
    gains: LTVDiffDriveGains
) {

    constructor(
        kX: Double,
        kY_0: Double,
        kvPlus_0: Double,
        kvMinus_0: Double,
        kY_1: Double,
        kTheta_1: Double,
        kvPlus_1: Double,
        kinematics: DifferentialDriveKinematics
    ) : this(LTVDiffDriveGains(
            kX, kY_0, kvPlus_0, kvMinus_0, kY_1, kTheta_1, kvPlus_1, kinematics
    ))

    private val kX = gains.kX
    private val kY_0 = gains.kY_0
    private val kvPlus_0 = gains.kvPlus_0
    private val kvMinus_0 = gains.kvMinus_0
    private val kY_1 = gains.kY_1
    private val kTheta_1 = gains.kTheta_1
    private val kvPlus_1 = gains.kvPlus_1
    private val kinematics = gains.kinematics

    private var poseError = Pose2d()

    fun calculate(
        currentPose: Pose2d,
        poseRef: Pose2d,
        referenceSpeeds: DifferentialDriveWheelSpeeds,
        currentSpeeds: DifferentialDriveWheelSpeeds,
        curvatureRadPerMeter: Double
    ): ChassisSpeeds {
        this.poseError = poseRef.relativeTo(currentPose)
        val eX = this.poseError.translation.x
        val eY = this.poseError.translation.y
        val eTheta = this.poseError.rotation.radians
        val eLeftVel = referenceSpeeds.leftMetersPerSecond - currentSpeeds.leftMetersPerSecond
        val eRightVel = referenceSpeeds.rightMetersPerSecond - currentSpeeds.rightMetersPerSecond

        val error = SimpleMatrix(3, 1, false, doubleArrayOf(
                eX, eY, eTheta, eLeftVel, eRightVel
        ))

        val referenceVelocity = (referenceSpeeds.leftMetersPerSecond + referenceSpeeds.rightMetersPerSecond) / 2.0
        val currentVelocity = (currentSpeeds.leftMetersPerSecond + currentSpeeds.rightMetersPerSecond) / 2.0
        val u = K(currentVelocity).mult(error)

        val toRet = ChassisSpeeds(u[0] + referenceVelocity, 0.0,
                u[1] + curvatureRadPerMeter * referenceVelocity)

        println("Commanding linear ${toRet.vxMetersPerSecond.meters.inFeet()} angluar ${toRet.omegaRadiansPerSecond.radians.inDegrees()}")

        return toRet
    }

    private fun K(v: Double) = SimpleMatrix(2, 5, true, doubleArrayOf(
            kX, k1_2(v).withSign(v), kTheta_1 * sqrt(v.absoluteValue), k1_4(v), k2_4(v),
            kX, -k1_2(v).withSign(v), -kTheta_1 * sqrt(v.absoluteValue), k2_4(v), k1_4(v)
    ))

    private fun k1_2(v: Double) = kY_0 + (kY_1 - kY_0).withSign(v)
    private fun k1_4(v: Double) = kvPlus_0 + (kvPlus_1 - kvPlus_0) * sqrt(v.absoluteValue)
    private fun k2_4(v: Double) = kvMinus_0 - (kvPlus_1 - kvPlus_0) * sqrt(v.absoluteValue)

    fun calculate(currentPose: Pose2d, currentSpeeds: DifferentialDriveWheelSpeeds, desiredState: Trajectory.State) =
            calculate(currentPose, desiredState.poseMeters,
                    kinematics.toWheelSpeeds(ChassisSpeeds(desiredState.velocityMetersPerSecond, 0.0,
                            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter)),
                    currentSpeeds, desiredState.curvatureRadPerMeter
            )
}
