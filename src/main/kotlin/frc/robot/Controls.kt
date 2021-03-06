package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.auto.routines.TestRoutine
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.climb.ClimbWithElevatorRetracted
import frc.robot.subsystems.climb.RetractStiltCommand
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.OpenLoopVisionDriveCommand
import frc.robot.subsystems.drive.SecondClosedVisionDriveCommand
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports
import org.team5940.pantry.lib.Updatable

object Controls : Updatable {

    var isClimbing = false
    var wantsHab3Mode = false

    private val zero = ZeroSuperStructureRoutine()

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {
        registerEmergencyMode()

        button(kX).changeOn(TestRoutine()())
//        button(kY).changeOn(DriveSubsystem.followTrajectory(TrajectoryFactory.testTrajectory2).beforeStarting {
//            DriveSubsystem.robotPosition = TrajectoryFactory.testTrajectory2.states.first().poseMeters
//        })

        // Shifting
        if (Constants.kIsRocketLeague) {
//            button(kBumperRight).change(OpenLoopVisionDriveCommand(true))
//            button(kBumperRight).change(ClosedLoopVisionDriveCommand(true))
//            button(kY).change(OpenLoopVisionDriveCommand(true, true))
//            button(kB).change(OpenLoopVisionDriveCommand(true, false))

//            button(kBumperRight).change(OpenLoopVisionDriveCommand(true, false))
//            button(kBumperRight).change(ClosedLoopVisionDriveCommand(true, false))
            button(kBumperRight).change(SecondClosedVisionDriveCommand(true, false))

            button(9).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }

//            val cargoCommand = sequential { +Superstructure.kCargoIntake; +IntakeCargoCommand(releasing = false) }
//            button(10).changeOff{ Superstructure.kStowed.schedule() }.change(cargoCommand)
            button(10).change(StartEndCommand(Runnable { IntakeSubsystem.hatchMotorOutput = 6.volt },
                    Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem))
        } else {
            triggerAxisButton(GenericHID.Hand.kRight).change(OpenLoopVisionDriveCommand(true))
            button(kBumperLeft).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }
        }

        state({ isClimbing && !wantsHab3Mode }) {
            pov(0).changeOn(ClimbSubsystem.hab2ClimbCommand)
            pov(90).changeOn(ClimbSubsystem.hab2ClimbCommand)
            pov(180).changeOn(ClimbSubsystem.hab2ClimbCommand)
            pov(280).changeOn(ClimbSubsystem.hab2ClimbCommand)
        }
        state({ isClimbing && wantsHab3Mode }) {
            pov(0).changeOn(ClimbSubsystem.hab3ClimbCommand)
            pov(90).changeOn(ClimbSubsystem.hab3ClimbCommand)
            pov(180).changeOn(ClimbSubsystem.hab3ClimbCommand)
            pov(270).changeOn(ClimbSubsystem.hab3ClimbCommand)
        }
        state({ isClimbing }) {
            // retract stuff
            button(kY).changeOn(ClimbWithElevatorRetracted())
            button(kA).changeOn(RetractStiltCommand())
        }

        // succcccc
        state({!isClimbing}) {
            greaterThanAxisButton(5, 0.5).changeOn { IntakeSubsystem.hatchMotorOutput = -12.volt; IntakeSubsystem.cargoMotorOutput = 12.volt }
                    .changeOff { IntakeSubsystem.hatchMotorOutput = 0.volt; IntakeSubsystem.cargoMotorOutput = 0.volt }
            lessThanAxisButton(5, 0.5).changeOn { IntakeSubsystem.hatchMotorOutput = 12.volt; IntakeSubsystem.cargoMotorOutput = -12.volt }
                    .changeOff { IntakeSubsystem.hatchMotorOutput = 0.volt; IntakeSubsystem.cargoMotorOutput = 0.volt }
        }

        // get both the buttons that are close together
//        pov(270+45).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
//        pov(45).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
//        pov(0).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
//
//        pov(135).changeOn(ClimbSubsystem.prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }
//        pov(180).changeOn(ClimbSubsystem.prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }
//        pov(180+45).changeOn(ClimbSubsystem.prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }
    }

    val operatorWPIJoystick = XboxController(1)
    val operatorFalconXbox = operatorWPIJoystick.mapControls {
        button(kX).changeOn(Superstructure.kCargoLow) // .changeOff { Superstructure.kStowed.schedule() }
        button(kY).changeOn(Superstructure.kCargoMid) // .changeOff { Superstructure.kStowed.schedule() }
        button(kB).changeOn(Superstructure.kCargoHigh) // .changeOff { Superstructure.kStowed.schedule() }
        button(kA).changeOn(Superstructure.kCargoShip) // .changeOff { Superstructure.kStowed.schedule() }
        // pov(0).change(ClosedLoopElevatorMove{Elevator.currentState.position + 1.inch})
        // hatch presets
        pov(270).changeOn(Superstructure.kHatchLow) // .changeOff { Superstructure.kStowed.schedule() }
        pov(0).changeOn(Superstructure.kHatchMid) // .changeOff { Superstructure.kStowed.schedule() }
        pov(90).changeOn(Superstructure.kHatchHigh) // .changeOff { Superstructure.kStowed.schedule() }
        pov(180).changeOn(Superstructure.kStowed)

        // hab climb
        button(kStart).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
        button(kBack).changeOn(ClimbSubsystem.hab2prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }

        // Left Stick Button
        button(9).changeOn(Superstructure.kStowed)

        // jogging
        state({!isClimbing}) {
            lessThanAxisButton(1, 0.8).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position + 1.inch })
            greaterThanAxisButton(1, 0.8).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position - 1.inch })
        }
        state({ isClimbing}) {
            lessThanAxisButton(1, 0.8).changeOn { ClimbSubsystem
                    .currentHab3Offset = (ClimbSubsystem.currentHab3Offset + 0.5.inches).coerceIn((-2).inches, 2.inches) }
            greaterThanAxisButton(1, 0.8).changeOn { ClimbSubsystem
                    .currentHab3Offset = (ClimbSubsystem.currentHab3Offset - 0.5.inches).coerceIn((-2).inches, 2.inches) }
        }
        lessThanAxisButton(0, 0.8).changeOn { Wrist.offset += 4.degrees }
        greaterThanAxisButton(0, 0.8).changeOn { Wrist.offset -= 4.degrees }

        // boring intake
        lessThanAxisButton(5).change(IntakeHatchCommand(releasing = true, shouldMutateArms = false))
        greaterThanAxisButton(5).change(
                StartEndCommand(Runnable { IntakeSubsystem.hatchMotorOutput = 3.volt; IntakeSubsystem.wantsOpen = true },
                        Runnable { IntakeSubsystem.setNeutral() })
        )

        // intake hatches
        val poked = Superstructure.kPokedStowed
        val stowed = Superstructure.kStowed
        val intake = IntakeHatchCommand(false)
        val delayedYote = StartEndCommand(Runnable { IntakeSubsystem.hatchMotorOutput = 8.volt
            IntakeSubsystem.wantsOpen = true },
                Runnable { IntakeSubsystem.setNeutral() }).withTimeout(0.75)
        triggerAxisButton(GenericHID.Hand.kLeft).changeOn { delayedYote.cancel(); poked.schedule(); intake.schedule() }
                .changeOff { poked.cancel(); intake.cancel(); stowed.schedule(); delayedYote.schedule()
        }.changeOn(IntakeHatchCommand(releasing = false))

        // outtake hatches
        triggerAxisButton(GenericHID.Hand.kRight).change(IntakeHatchCommand(true))

        // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
        // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
        val cargoCommand = parallel { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake.beforeStarting { IntakeSubsystem.wantsOpen = true }; +IntakeCargoCommand(releasing = false) }
        button(kBumperLeft).changeOff { Superstructure.kStowed.schedule() }.change(cargoCommand)
        button(kBumperRight).change(IntakeCargoCommand(true))
    }

//    val operatorJoy = Joystick(5)
//    val operatorFalconHID = operatorJoy.mapControls {
//        // cargo presets
// //            button(12).changeOn(Superstructure.kCargoIntake.andThen { IntakeSubsystem.wantsOpen = true }) // .changeOff { Superstructure.kStowed.schedule() }
//        button(7).changeOn(Superstructure.kCargoLow) // .changeOff { Superstructure.kStowed.schedule() }
//        button(6).changeOn(Superstructure.kCargoMid) // .changeOff { Superstructure.kStowed.schedule() }
//        button(5).changeOn(Superstructure.kCargoHigh) // .changeOff { Superstructure.kStowed.schedule() }
//        button(8).changeOn(Superstructure.kCargoShip) // .changeOff { Superstructure.kStowed.schedule() }
//
//        // hatch presets
//        button(3).changeOn(Superstructure.kHatchLow) // .changeOff { Superstructure.kStowed.schedule() }
//        button(2).changeOn(Superstructure.kHatchMid) // .changeOff { Superstructure.kStowed.schedule() }
//        button(1).changeOn(Superstructure.kHatchHigh) // .changeOff { Superstructure.kStowed.schedule() }
//
//        // Stow (for now like this coz i dont wanna break anything
//        button(10).changeOn(Superstructure.kStowed)
//
//        button(9).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position + 1.inch })
//        button(11).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position - 1.inch })
//
//        // that one passthrough preset that doesnt snap back to normal
// //            button(4).changeOn(Superstructure.kBackHatchFromLoadingStation)
//
//        // intake hatches
//        val poked = Superstructure.kPokedStowed
//        val stowed = Superstructure.kStowed
//        val intake = IntakeHatchCommand(false)
//        val delayedYote = StartEndCommand(Runnable{IntakeSubsystem.hatchMotorOutput = 8.volt
//            IntakeSubsystem.wantsOpen = true},
//                Runnable{IntakeSubsystem.setNeutral()}).withTimeout(0.75)
//        lessThanAxisButton(1).changeOn { delayedYote.cancel(); poked.schedule(); intake.schedule() }
//                .changeOff { poked.cancel(); intake.cancel(); stowed.schedule(); delayedYote.schedule()
//        }.changeOn(IntakeHatchCommand(releasing = false))
//
//        greaterThanAxisButton(1).change(IntakeHatchCommand(releasing = true))
//
//        // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
//        // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
//        val cargoCommand = sequential { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake.beforeStarting { IntakeSubsystem.wantsOpen = true }; +IntakeCargoCommand(releasing = false) }
//        lessThanAxisButton(0).changeOff { (sequential { +ClosedLoopWristMove(40.degree) ; +Superstructure.kStowed; }).schedule() }.change(cargoCommand)
//        greaterThanAxisButton(0).changeOff { }.change(IntakeCargoCommand(true))
//
//        button(4).changeOn(ClimbSubsystem.prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }
//        state({ isClimbing && !wantsHab3Mode }) {
//            button(12).changeOn(ClimbSubsystem.hab2ClimbCommand)
//        }
//        state({ isClimbing && wantsHab3Mode }) {
//            button(12).changeOn(ClimbSubsystem.hab3ClimbCommand)
//        }
//
//    }

    override fun update() {
        driverFalconXbox.update()
//        operatorFalconHID.update()
        operatorFalconXbox.update()
    }
}

private fun Command.beforeStarting(function: () -> Unit) = this.beforeStarting(Runnable(function))

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }

private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
//        Robot.activateEmergency()
        val command = object : FalconCommand(Superstructure, DriveSubsystem, Elevator, Proximal, Wrist, IntakeSubsystem) {
            override fun execute() {
                Superstructure.setNeutral()
                Elevator.setNeutral()
                Proximal.setNeutral()
                Wrist.setNeutral()
                IntakeSubsystem.setNeutral()
                DriveSubsystem.setNeutral()
                DriveSubsystem.leftMotor.setClosedLoopGains()
                DriveSubsystem.rightMotor.setClosedLoopGains()
            }
        }.withTimeout(0.5)
        command.schedule()
    }
    button(kStart).changeOn {
//        Robot.recoverFromEmergency()
    }
}
