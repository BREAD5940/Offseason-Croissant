package frc.robot.subsystems.drive

import frc.robot.Constants
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.subsystems.superstructure.Length
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.utils.Source
import java.awt.Color

/**
 * Command to follow a smooth trajectory using a trajectory following controller
 *
 * @param trajectorySource Source that contains the trajectory to follow.
 */
class VisionAssistedTrajectoryTracker(
    val trajectorySource: Source<Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>>,
    private val radiusFromEnd: Length,
    private val useAbsoluteVision: Boolean = false
) : FalconCommand(DriveSubsystem) {

//    private var visionFinished = false

    private var prevError = 0.0

    @Suppress("LateinitUsage")
    private lateinit var trajectory: Trajectory<SIUnit<Second>, TimedEntry<Pose2dWithCurvature>>
    private var lastOutput = TrajectoryTrackerOutput(0.feet.velocity, 0.feet.acceleration, 0.degree.velocity, 0.degree.acceleration)

    override fun isFinished() = DriveSubsystem.trajectoryTracker.isFinished // visionFinished

    private var lastKnownTargetPose: Pose2d? = null
    private var shouldVision = false

    /**
     * Reset the trajectory follower with the new trajectory.
     */
    override fun initialize() {
        trajectory = trajectorySource()
        DriveSubsystem.trajectoryTracker.reset(trajectory)
        LiveDashboard.isFollowingPath = true
        lastKnownTargetPose = null
//        visionFinished = false
        println("VISION INIT")
        LimeLight.wantedPipeline = 1
    }

    var lastAbsoluteAngle: SIUnit<Radian>? = null

    override fun execute() {
        val robotPositionWithIntakeOffset = DriveSubsystem.robotPosition // IntakeSubsystem.robotPositionWithIntakeOffset

        val nextState = DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition)

        val withinVisionRadius =
                robotPositionWithIntakeOffset.translation.distance(trajectory.lastState.state.pose.translation) <
                        radiusFromEnd.meter

        if (withinVisionRadius) {
            LEDs.wantedState = LEDs.State.Off

            val newTarget = if (!useAbsoluteVision) {
                TargetTracker.getBestTarget(!trajectory.reversed)
            } else {
                val reference = if (!trajectory.reversed) Constants.kCenterToForwardIntakeStowed else Constants.kBackwardIntakeToCenter
                TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + reference).translation)
            }

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null) {
            visionActive = true
            val transform = lastKnownTargetPose inFrameOfReferenceOf robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val error = LimeLight.lastYaw.radian //(angle + if (!trajectory.reversed) Rotation2d() else Math.PI.radian.toRotation2d()).radian

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

            println("angle error ${error.radian.degree} turn $turn")

            DriveSubsystem.setOutput(TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    SIUnit((nextState.linearVelocity - lastOutput.linearVelocity).value / 0.020),
                    turn.radian.velocity,
                    0.radian.acceleration))//SIUnit((turn.radian.velocity - lastOutput.angularVelocity).value / 0.020)))
            lastOutput = nextState

            prevError = error
        } else {
            DriveSubsystem.setOutput(TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    SIUnit((nextState.linearVelocity - lastOutput.linearVelocity).value / 0.020),
                    nextState.angularVelocity,
                    SIUnit((nextState.angularVelocity - lastOutput.angularVelocity).value / 0.020)))
            lastOutput = nextState
        }

        val referencePoint = DriveSubsystem.trajectoryTracker.referencePoint
        if (referencePoint != null) {
            val referencePose = referencePoint.state.state.pose

            // Update Current Path Location on Live Dashboard
            LiveDashboard.pathX = referencePose.translation.x.feet
            LiveDashboard.pathY = referencePose.translation.y.feet
            LiveDashboard.pathHeading = referencePose.rotation.radian
        }
    }

    /**
     * Make sure that the drivetrain is stopped at the end of the command.
     */
    override fun end(interrupted: Boolean) {
        DriveSubsystem.zeroOutputs()
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