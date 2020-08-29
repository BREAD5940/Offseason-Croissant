package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.Constants
import frc.robot.Network
import frc.robot.auto.paths.plus
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.subsystems.superstructure.Length
import frc.robot.vision.TargetTracker
import java.awt.Color
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.utils.Source

// private val Trajectory.reversed get() =
private val Trajectory.lastState get() = states.last()

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class VisionAssistedTrajectoryTracker(
    val trajectorySource: Source<Trajectory>,
    private val radiusFromEnd: Length,
    private val useAbsoluteVision: Boolean = false
) : FalconCommand(DriveSubsystem) {

//    private var visionFinished = false

    private var prevError = 0.0

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory
    private var reversed: Boolean = false
//    private var lastOutput = TrajectoryTrackerOutput(0.feet.velocity, 0.feet.acceleration, 0.degrees.velocity, 0.degrees.acceleration)
    private var lastLinearVelocity = 0.meters.velocity
    private var lastAngularVelocity = 0.radians.velocity
    private var trajectoryFinished = false

    override fun isFinished() = trajectoryFinished // DriveSubsystem.trajectoryTracker.isFinished // visionFinished

    private var lastKnownTargetPose: Pose2d? = null
    private var shouldVision = false

    private val timer = Timer()

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        trajectory = trajectorySource()
        reversed = trajectory.states[(trajectory.states.size / 2.0).toInt()].velocityMetersPerSecond >= 0.0
//        DriveSubsystem.controller.reset(trajectory)
        LiveDashboard.isFollowingPath = true
        lastKnownTargetPose = null
//        visionFinished = false
        println("VISION INIT")
        LimeLight.wantedPipeline = 1
        trajectoryFinished = false
        timer.reset()
        timer.start()
    }

    var lastAbsoluteAngle: SIUnit<Radian>? = null

    override fun execute() {
        val robotPositionWithIntakeOffset = DriveSubsystem.robotPosition // IntakeSubsystem.robotPositionWithIntakeOffset

        val samplePoint = trajectory.sample(timer.get())
        val nextState = DriveSubsystem.controller.calculate(DriveSubsystem.robotPosition, samplePoint)

        val withinVisionRadius =
                robotPositionWithIntakeOffset.translation.getDistance(trajectory.lastState.poseMeters.translation) <
                        radiusFromEnd.inMeters()

        if (withinVisionRadius) {
            LEDs.wantedState = LEDs.State.Off

            val newTarget = if (!useAbsoluteVision) {
                TargetTracker.getBestTarget(!reversed)
            } else {
                val reference = if (!reversed) Constants.kCenterToForwardIntakeStowed else Constants.kBackwardIntakeToCenter
                TargetTracker.getAbsoluteTarget((trajectory.lastState.poseMeters + reference).translation)
            }

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null) {
            visionActive = true
            val transform = lastKnownTargetPose.relativeTo(robotPositionWithIntakeOffset)
            val angle = Rotation2d(transform.translation.x, transform.translation.y)

            Network.visionDriveAngle.setDouble(angle.degrees)
            Network.visionDriveActive.setBoolean(true)

            val error = LimeLight.lastYaw.radian // (angle + if (!trajectory.reversed) Rotation2d() else Math.PI.radian.toRotation2d()).radian

            // at 0 speed this should be 1, and at 10ft/sec it should be 2
            // so (0, 1) and (10, 2)
            // y = (2-1)/(10-0) * (x - 0) + 1
//            val velocity = (with(DriveSubsystem) {
//                leftMotor.encoder.velocity + rightMotor.encoder.velocity
//            }).absoluteValue / 2.0
//            val scaler = velocity.value * (/* max scaler */ 4.0 - /* min scaler */ 1.0) /
//                    (/* velocity at max scaler */10.feet.meter) + 1.0
//            var kp = (kCorrectionKp * scaler)
//            if (kp > 0.7) kp = 0.7
            val kp = kCorrectionKp

            val multiplier = if (DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter
            val turn = /* kCorrectionKp */ kp * error * multiplier + kCorrectionKd * (error - prevError) * multiplier

            println("angle error ${error.radians.degree} turn $turn")

            DriveSubsystem.setTrajectoryOutput(
                    nextState.vxMetersPerSecond.meters.velocity,
                    SIUnit((nextState.vxMetersPerSecond - lastLinearVelocity.value) / 0.020),
                    turn.radians.velocity,
                    0.radian.acceleration) // SIUnit((turn.radian.velocity - lastOutput.angularVelocity).value / 0.020)))

            lastLinearVelocity = nextState.vxMetersPerSecond.meters.velocity
            lastAngularVelocity = turn.radians.velocity

            prevError = error
        } else {
            DriveSubsystem.setTrajectoryOutput(
                    nextState.vxMetersPerSecond.meters.velocity,
                    SIUnit((nextState.vxMetersPerSecond - lastLinearVelocity.value) / 0.020),
                    nextState.omegaRadiansPerSecond.radians.velocity,
                    SIUnit((nextState.omegaRadiansPerSecond - lastAngularVelocity.value) / 0.020))

            lastLinearVelocity = nextState.vxMetersPerSecond.meters.velocity
            lastAngularVelocity = nextState.omegaRadiansPerSecond.radians.velocity
        }

//        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (samplePoint != null) {
            val referencePose = samplePoint.poseMeters

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x / kFeetToMeter
            LiveDashboard.pathY = referencePose.translation.y / kFeetToMeter
            LiveDashboard.pathHeading = referencePose.rotation.radians
        }
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
        LiveDashboard.isFollowingPath = false
        visionActive = false
        shouldVision = false
        LEDs.wantedState = LEDs.State.Solid(Color.red)
    }

    companion object {
        const val kCorrectionKp = 2.5 // 5.5 * 2.0
        const val kCorrectionKd = 0.0 // 5.0
        var visionActive = false
    }
}
