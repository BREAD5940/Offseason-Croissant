package frc.robot.subsystems.drive

import frc.robot.Constants
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Length
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.utils.Source

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
//        visionFinished = false
        println("VISION INIT")
    }

    var lastAbsoluteAngle: SIUnit<Radian>? = null

    override fun execute() {
        val robotPositionWithIntakeOffset = DriveSubsystem.robotPosition // IntakeSubsystem.robotPositionWithIntakeOffset

        val nextState = DriveSubsystem.trajectoryTracker.nextState(DriveSubsystem.robotPosition)

        val withinVisionRadius =
                robotPositionWithIntakeOffset.translation.distance(trajectory.lastState.state.pose.translation) <
                        radiusFromEnd.meter

        if (withinVisionRadius) {
            val newTarget = if (!useAbsoluteVision) {
                TargetTracker.getBestTarget(!trajectory.reversed)
            } else {
                val reference = if(!trajectory.reversed) Constants.kCenterToForwardIntakeStowed else Constants.kBackwardIntakeToCenter
                TargetTracker.getAbsoluteTarget((trajectory.lastState.state.pose + reference).translation)
            }

            val newPose = newTarget?.averagedPose2d
            if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose
        }

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose != null) {
            visionActive = true
            val transform = lastKnownTargetPose inFrameOfReferenceOf robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val error = (angle + if (!trajectory.reversed) Rotation2d() else Math.PI.radian.toRotation2d()).radian
            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)

            DriveSubsystem.setOutput(TrajectoryTrackerOutput(
                    nextState.linearVelocity,
                    SIUnit((nextState.linearVelocity - lastOutput.linearVelocity).value / 0.020),
                    turn.radian.velocity,
                    SIUnit((turn.radian.velocity - lastOutput.angularVelocity).value / 0.020)))
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
    }

    companion object {
        const val kCorrectionKp = 1.8 * 5.0 // 5.5 * 2.0
        const val kCorrectionKd = 50.0 // 5.0
        var visionActive = false
    }
}