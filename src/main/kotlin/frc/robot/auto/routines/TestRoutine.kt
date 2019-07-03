//package frc.robot.auto.routines
//
//import edu.wpi.first.wpilibj.experimental.command.InstantCommand
//import frc.robot.auto.paths.TrajectoryFactory
//import frc.robot.subsystems.drive.DriveSubsystem
//import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
//import org.ghrobotics.lib.mathematics.units.feet
//import org.ghrobotics.lib.mathematics.units.second
//
//class TestRoutine : AutoRoutine() {
//
//    init {
//
//        +InstantCommand(Runnable{
//            DriveSubsystem.localization.reset(TrajectoryFactory.tenFootTest.firstState.state.pose)
//        })
//
////        +JankyGoToState(RobotConfig.auto.fieldPositions.hatchLowGoal, SuperStructure.iPosition.HATCH)
//
//        +parallel(
//
//                followVisionAssistedTrajectory(TrajectoryFactory.tenFootTest, { false }, 4.feet, false),
//
//                RunIntake.grabHatchFor(TrajectoryFactory.tenFootTest.duration)
//
//        )
//
//        +parallel(
//                DrivePower(-0.3, 1.0),
//                RunIntake.grabHatchFor(0.5.second)
//        )
//    }
//}