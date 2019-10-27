package frc.robot.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second

class TestRoutine : AutoRoutine() {

    override val duration = 0.second

//    override val routine = sequential {
//        +InstantCommand(Runnable{
//            DriveSubsystem.robotPosition = TrajectoryFactory.testTrajectory.firstState.state.pose
//            DriveSubsystem.lowGear = false
//        })
//        +DriveSubsystem.followTrajectory(
//                TrajectoryFactory.testTrajectory
//        )
//    }

    override val routine = sequential {
        // place the hatch
        +super.followVisionAssistedTrajectory(
                TrajectoryFactory.cargoS1PrepToCargoS1,
                Autonomous.isStartingOnLeft,
                100.feet,
                false
        ).beforeStarting {
            DriveSubsystem.robotPosition = TrajectoryFactory.cargoS1PrepToCargoS1.firstState.state.pose
        }
    }
}