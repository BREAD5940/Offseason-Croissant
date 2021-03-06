package frc.robot.auto.routines

import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.auto.Autonomous
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.paths.TrajectoryWaypoints
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelRace
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class HybridRoutine : AutoRoutine() {

    val path1 = TrajectoryFactory.sideStartToCargoShipS1Prep
    val path2 = TrajectoryFactory.cargoShipS1ToS1Prep
    val path3 = TrajectoryFactory.cargoS1PrepToLoadingStation
    val path4 = TrajectoryFactory.loadingStationToRocketFPrep
    val path5 = TrajectoryFactory.rocketFPrepareToRocketF

    private val pathMirrored = Autonomous.isStartingOnLeft

    override val duration: SIUnit<Second> = 0.second
//        get() = path1.duration + path2.duration + path3.duration

    override val routine
        get() = sequential {

            +PrintCommand("Starting")
            +InstantCommand(Runnable { DriveSubsystem.lowGear = false })

//            +parallel {
//                +followVisionAssistedTrajectory(path1, pathMirrored, 3.feet)
//                +sequential {
//                    +DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform)
//                    +WaitCommand(0.5)
//                    +Superstructure.kMatchStartToStowed
//                }
//            }

            +parallel {
                +sequential {
                    +DriveSubsystem.followTrajectory(
                            path1,
                            Autonomous.isStartingOnLeft
                    )
                    +PointTurnCommand {
                        90.degrees.toRotation2d() * if (Autonomous.isStartingOnLeft()) -1.0 else 1.0
                    }
                    // drive forward
                    +super.followVisionAssistedTrajectory(
                            TrajectoryFactory.cargoS1PrepToCargoS1,
                            Autonomous.isStartingOnLeft,
                            100.feet,
                            false
                    )
                }
                +(sequential {
                    +DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform)
                    +WaitCommand(0.5)
                    +Superstructure.kMatchStartToStowed
//                    +Superstructure.kPokedStowed
                })
                +sequential {
                    +IntakeHatchCommand(false).withTimeout(0.5)
                }
            }

            val path2_ = DriveSubsystem.followTrajectory(path2, pathMirrored)
            val path3_ = followVisionAssistedTrajectory(path3, pathMirrored, 5.feet)

            // back up and turn around
//            +parallel {
//                +path2_
//                +IntakeHatchCommand(true).withTimeout(1.second)
//            }
//
//            +parallel {

//                +sequential {
//                    +path2_
//                    +parallel {
//                        +path3_
//                        +Superstructure.kPokedStowed
//                    }
//                }
//                +sequential {
//                    +IntakeHatchCommand(true).withTimeout(1.5)
//                    +WaitCommand(path3.duration.second + path2.duration.second - 4)
//                    +IntakeHatchCommand(false).withExit { path3_.isFinished }
//                }.withExit { path3_.isFinished }
//            }

            +parallelRace { +path2_; +IntakeHatchCommand(true) }
            +parallel {
                +path3_
                +Superstructure.kPokedStowed
                +sequential {
                    +WaitCommand(path3.totalTimeSeconds - 2.5)
                    +IntakeHatchCommand(false).withExit { path3_.isFinished }
                }
            }

            +relocalize(TrajectoryWaypoints.kLoadingStationReversed, true, pathMirrored, isStowed = true, isPoked = true)

            +parallel {
                +IntakeHatchCommand(false).withTimeout(1.0).beforeStarting { IntakeSubsystem.wantsOpen = true }
                +Superstructure.kStowed
                +DriveSubsystem.followTrajectory(path4, pathMirrored)
            }
            +PointTurnCommand { (-143 - 8).degrees.toRotation2d() * if (Autonomous.isStartingOnLeft()) -1.0 else 1.0 }

            +super.followVisionAssistedTrajectory(
                    path5,
                    pathMirrored,
                    100.feet,
                    false
            )

            +parallel {
                +IntakeHatchCommand(true).withTimeout(1.0)
                +RunCommand(Runnable { DriveSubsystem.setPercent(-0.3, -0.3) }).withTimeout(1.0)
            }
        }
}
