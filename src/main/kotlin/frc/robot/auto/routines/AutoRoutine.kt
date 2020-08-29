package frc.robot.auto.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.plus
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionAssistedTrajectoryTracker
import frc.robot.subsystems.superstructure.Length
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
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
            DriveSubsystem.odometry.resetPosition(Autonomous.startingPosition().pose, DriveSubsystem.gyro())
            println("[AutoRoutine] Disabling compressor...")
            DriveSubsystem.compressor.stop()
        })
        +routine
    }.raceWith(WaitUntilCommand { Robot.sensorlessModeActive })).andThen (Runnable{ DriveSubsystem.compressor.start() }) as CommandBase

    fun followVisionAssistedTrajectory(
        originalTrajectory: Trajectory,
        pathMirrored: BooleanSource,
        radiusFromEnd: Length,
        useAbsoluteVision: Boolean = false
    ) = VisionAssistedTrajectoryTracker(
            pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
            radiusFromEnd,
            useAbsoluteVision
    )

    protected fun relocalize(position: Pose2d, forward: Boolean, pathMirrored: BooleanSource, isStowed: Boolean = true, isPoked: Boolean = false) = InstantCommand(Runnable {
        val offset = if (forward) {
            if (isStowed && isPoked) {
                Constants.kForwardIntakePokedStowedToCenter
            } else if (isStowed && !isPoked) {
                Constants.kForwardIntakeStowedToCenter
            } else Constants.kForwardIntakeToCenter
        } else {
            Constants.kBackwardIntakeToCenter
        }

        val newPosition = Pose2d(
                pathMirrored.map(position.mirror(), position)().translation, // if pathMirrored is true, mirror the pose
                // otherwise, don't. Use that translation2d for the new position
                DriveSubsystem.robotPosition.rotation
        ) + offset
        println("RESETTING LOCALIZATION TO ${newPosition.asString()}")
        DriveSubsystem.odometry.resetPosition(newPosition, DriveSubsystem.gyro())
    })

    protected fun executeFor(time: SIUnit<Second>, command: FalconCommand) = sequential {
        +command
        +WaitCommand(100.0)
    }.withTimeout(time)

    private fun Pose2d.asString() = "Pose X:${translation.x / kFeetToMeter}\' Y:${translation.y / kFeetToMeter}' Theta:${rotation.degrees}deg"

    fun notWithinRegion(region: Rectangle2d) = object : CommandBase() {
        override fun isFinished() = !region.contains(DriveSubsystem.robotPosition.translation)
    }

    operator fun Command.unaryPlus() {
        addCommands(this@unaryPlus)
    }
}

fun Command.withExit(exit: BooleanSource): Command = this.withInterrupt(exit)

// fun Command.withExit(exit: BooleanSource) = parallelRace {
//    +this@withExit
//    +WaitUntilCommand(exit)
// }

fun Command.withTimeout(second: SIUnit<Second>): Command = this.withTimeout(second.inSeconds())

// val Translation2d.mirror get() = Translation2d(this.x, 27.0.feet.inMeters() - this.y)

fun Command.beforeStarting(function: () -> Unit) = this.beforeStarting(Runnable(function))

fun Command.andThen(function: () -> Unit) = this.andThen(Runnable(function))
