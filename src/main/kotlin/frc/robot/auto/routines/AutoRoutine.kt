package frc.robot.auto.routines

import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.auto.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionAssistedTrajectoryTracker
import frc.robot.subsystems.superstructure.Length
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map

abstract class AutoRoutine : SequentialCommandGroup(), Source<Command> {

    abstract val duration: SIUnit<Second>
    abstract val routine: Command

    override fun invoke() = (sequential {
        +InstantCommand(Runnable {
            println("[AutoRoutine] Starting routine...")
            DriveSubsystem.localization.reset(Autonomous.startingPosition().pose)
            println("[AutoRoutine] Disabling compressor...")
            DriveSubsystem.compressor.stop()
        })
        +routine
    }.raceWith(WaitUntilCommand { Robot.emergencyActive })).whenFinished { DriveSubsystem.compressor.start() } as CommandBase

    fun followVisionAssistedTrajectory(
        originalTrajectory: TimedTrajectory<Pose2dWithCurvature>,
        pathMirrored: BooleanSource,
        radiusFromEnd: Length,
        useAbsoluteVision: Boolean = false
    ) = VisionAssistedTrajectoryTracker(
            pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
            radiusFromEnd,
            useAbsoluteVision
    )

    protected fun relocalize(position: Pose2d, forward: Boolean, pathMirrored: BooleanSource, isStowed: Boolean = false) = InstantCommand(Runnable {
        val newPosition = Pose2d(
                pathMirrored.map(position.mirror, position)().translation, // if pathMirrored is true, mirror the pose
                // otherwise, don't. Use that translation2d for the new position
                DriveSubsystem.localization().rotation
        ) + if (forward) (if (isStowed) Constants.kForwardIntakeStowedToCenter else Constants.kForwardIntakeToCenter) else Constants.kBackwardIntakeToCenter
        println("RESETTING LOCALIZATION TO ${newPosition.asString()}")
        DriveSubsystem.localization.reset(newPosition)
    })

    protected fun executeFor(time: SIUnit<Second>, command: FalconCommand) = sequential {
        +command
        +WaitCommand(100.0)
    }.withTimeout(time)

    private fun Pose2d.asString() = "Pose X:${translation.x / kFeetToMeter}\' Y:${translation.y / kFeetToMeter}' Theta:${rotation.degree}deg"

    fun notWithinRegion(region: Rectangle2d) = object : CommandBase() {
        override fun isFinished() = !region.contains(DriveSubsystem.robotPosition.translation)
    }

    operator fun Command.unaryPlus() {
        addCommands(this@unaryPlus)
    }
}

fun Command.withExit(exit: BooleanSource): Command = this.interruptOn(exit)

// fun Command.withExit(exit: BooleanSource) = parallelRace {
//    +this@withExit
//    +WaitUntilCommand(exit)
// }

fun Command.withTimeout(second: SIUnit<Second>): Command = this.withTimeout(second.second)

val Translation2d.mirror get() = Translation2d(this.x, 27.0.feet - this.y)
