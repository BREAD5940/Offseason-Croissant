package frc.robot.auto.routines

import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.volt

class BottomRocketRoutine2 : AutoRoutine() {

    private val path1 = TrajectoryFactory.sideStartReversedToRocketFPrepare
    private val path2 = TrajectoryFactory.rocketFPrepareToRocketF

    // Second path goes to the loading station to pick up a hatch panel
    private val path3 = TrajectoryFactory.rocketFToRocketFPrepare
    private val path4 = TrajectoryFactory.rocketFPrepareToLoadingStation

    // Third path goes to the near side of the rocket
    private val path5 = TrajectoryFactory.loadingStationReversedToRocketNPrep
    private val path6 = TrajectoryFactory.rocketNPrepToRocketN

    override val duration: SIUnit<Second>
        get() = path4.duration + path5.duration + path1.duration + path2.duration

    override val routine
        get() = sequential {

            +PrintCommand("Starting")
            +InstantCommand(Runnable { DriveSubsystem.lowGear = false })

            +parallel {
                +DriveSubsystem.followTrajectory(
                        path1,
                        Autonomous.isStartingOnLeft
                )
                +(sequential {
                    +DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform)
                    +WaitCommand(0.5)
                    +Superstructure.kMatchStartToStowed
                }).beforeStarting { IntakeSubsystem.hatchMotorOutput = 6.volt; IntakeSubsystem.wantsOpen = true }.whenFinished { IntakeSubsystem.hatchMotorOutput = 0.volt }
            }

            +PointTurnCommand {

                    //                val goalTarget = TargetTracker.getBestTarget(true)
//                if(goalTarget != null) {
//                    val goal = goalTarget.averagedPose2d.translation
//                    val error = (goal - DriveSubsystem.robotPosition.translation)
//                    Rotation2d(error.x.meter, error.y.meter, true)
//                if (LimeLight.hasTarget) {
//                    // plus the rotation of the dt at that timestamp
//                    LimeLight.currentState.tx.toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation
//                } else {
//                    -151.degree.toRotation2d()
//                }
                (-143).degree.toRotation2d() * if (Autonomous.isStartingOnLeft()) -1.0 else 1.0
            }

//            +PointTurnCommand {
//                (LimeLight.currentState.tx.toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation) * if(Autonomous.isStartingOnLeft()) -1.0 else 1.0
//            }

            // place the hatch
            +super.followVisionAssistedTrajectory(
                    path2,
                    Autonomous.isStartingOnLeft,
                    10.feet,
                    false
            )

            // Reorient position on field based on Vision alignment.
            +relocalize(
                    TrajectoryWaypoints.kRocketF,
                    true,
                    Autonomous.isStartingOnLeft,
                    isStowed = true
            )

            val spline3 = DriveSubsystem.followTrajectory(
                    path3,
                    Autonomous.isStartingOnLeft
            )
            val spline4 = super.followVisionAssistedTrajectory(
                    path4,
                    Autonomous.isStartingOnLeft,
                    5.feet, false
            )

            // Part 2: Place hatch and go to loading station.
            +parallel {
                // Follow the trajectory with vision correction to the loading station.
                +sequential {
                    +spline3
                    +spline4
                }

                // Take the superstructure to pickup position and arm hatch intake 3 seconds before arrival.
                +sequential {
                    // Place hatch panel.
                    +IntakeHatchCommand(true).withTimeout(1.second)
                    +WaitCommand(3.0)
                    +parallel {
                        +Superstructure.kPokedStowed
                        +IntakeHatchCommand(false).withExit { spline4.isFinished }
                    }
                }
            }

            // Reorient position on field based on Vision alignment.
            +relocalize(
                    TrajectoryWaypoints.kLoadingStationReversed,
                    true,
                    Autonomous.isStartingOnLeft,
                    isStowed = true, isPoked = true
            )

            // Part 3: Pickup hatch and go to the near side of the rocket.
            +parallel {
                val path = DriveSubsystem.followTrajectory(path5, Autonomous.isStartingOnLeft)
                +path
                // Make sure the intake is holding the hatch panel by opening the arms and succing it on
                +StartEndCommand(Runnable { IntakeSubsystem.hatchMotorOutput = 8.volt
                    IntakeSubsystem.wantsOpen = true },
                        Runnable { IntakeSubsystem.setNeutral() }).withTimeout(0.75)

                // retract the intake
                +Superstructure.kStowed
            }
            // turn to face the goal
            +PointTurnCommand {
                //                val goal = TrajectoryWaypoints.kRocketN.translation.let { if(Autonomous.isStartingOnLeft()) it.mirror else it }
//                val error = (goal - DriveSubsystem.robotPosition.translation)
//                Rotation2d(error.x.meter, error.y.meter, true)

//                if (LimeLight.hasTarget) {
//                    // plus the rotation of the dt at that timestamp
//                    LimeLight.currentState.tx.toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation
//                } else {
                -13.degree.toRotation2d() * if (Autonomous.isStartingOnLeft()) -1.0 else 1.0
//                }
//
// //                (-28.75).degree.toRotation2d()
//            }.perpetually().withExit { LimeLight.currentState.tx.absoluteValue < 2.degree }.withTimeout(3.0)
            }

//            +PointTurnCommand {
//                (LimeLight.currentState.tx.toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation) * if(Autonomous.isStartingOnLeft()) -1.0 else 1.0
//            }

            +WaitCommand(0.5)
            +followVisionAssistedTrajectory(
                    path6,
                    Autonomous.isStartingOnLeft,
                    4.feet
            )

            +parallel {
                +IntakeHatchCommand(true).withTimeout(1.0)
                +RunCommand(Runnable { DriveSubsystem.tankDrive(-0.3, -0.3) }).withTimeout(1.0)
            }
        }
}