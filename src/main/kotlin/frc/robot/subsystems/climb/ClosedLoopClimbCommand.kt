package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.auto.routines.AutoRoutine
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second

class ClosedLoopClimbCommand : AutoRoutine() {

    override val duration = 8.second

    override val routine = sequential {
        // take superstructure up to scoring height
        +SuperstructurePlanner.everythingMoveTo(30.inch, 0.degree, 0.degree) // TODO check preset
        // yote forward
        +InstantCommand(Runnable { DriveSubsystem.lowGear = true })
        +executeFor(2.second, object : FalconCommand(DriveSubsystem) {
            override fun execute() {
                DriveSubsystem.tankDrive(0.5, 0.5)
            }
        })
        // take superstructure to prop up position
            +SuperstructurePlanner.everythingMoveTo(30.inch, (-60).degree, 120.degree) // TODO check preset
        +SyncronizedLiftClimbCommand()
        // yeet forward again
        +executeFor(5.second, object : FalconCommand(DriveSubsystem, ClimbSubsystem) {
            override fun execute() {
                DriveSubsystem.tankDrive(0.5, 0.5)
                ClimbSubsystem.intakeWheels.setDutyCycle(0.5)
            }
        })
        // stow the elevator somehow
        +ClosedLoopElevatorMove(30.inch)
        +ClosedLoopWristMove(45.degree)
        +ClosedLoopProximalMove((-60).degree)
    }
}
