package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.* // ktlint-disable no-wildcard-imports
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports
import org.team5940.pantry.lib.Updatable

object Controls : Updatable {

    var isClimbing = false
    var wantsHab3Mode = false

    private val zero = ZeroSuperStructureRoutine()

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {
        registerEmergencyMode()

        // Shifting
        if (Constants.kIsRocketLeague) {
            button(kBumperRight).change(VisionDriveCommand(true))
            button(9).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }

//            val cargoCommand = sequential { +Superstructure.kCargoIntake; +IntakeCargoCommand(releasing = false) }
//            button(10).changeOff{ Superstructure.kStowed.schedule() }.change(cargoCommand)
        } else {
            triggerAxisButton(GenericHID.Hand.kRight).change(VisionDriveCommand(true))
            button(kBumperLeft).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }
        }

        // get both the buttons that are close together
        pov(90).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
        pov(45).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }
        pov(0).changeOn(ClimbSubsystem.hab3prepMove).changeOn { isClimbing = true; wantsHab3Mode = true }

        pov(180).changeOn(Superstructure.kStraightDown)

    }

    val operatorJoy = Joystick(5)
    val operatorFalconHID = operatorJoy.mapControls {
        // cargo presets
//            button(12).changeOn(Superstructure.kCargoIntake.andThen { Intake.wantsOpen = true }) // .changeOff { Superstructure.kStowed.schedule() }
        button(7).changeOn(Superstructure.kCargoLow) // .changeOff { Superstructure.kStowed.schedule() }
        button(6).changeOn(Superstructure.kCargoMid) // .changeOff { Superstructure.kStowed.schedule() }
        button(5).changeOn(Superstructure.kCargoHigh) // .changeOff { Superstructure.kStowed.schedule() }
        button(8).changeOn(Superstructure.kCargoShip) // .changeOff { Superstructure.kStowed.schedule() }

        // hatch presets
        button(3).changeOn(Superstructure.kHatchLow) // .changeOff { Superstructure.kStowed.schedule() }
        button(2).changeOn(Superstructure.kHatchMid) // .changeOff { Superstructure.kStowed.schedule() }
        button(1).changeOn(Superstructure.kHatchHigh) // .changeOff { Superstructure.kStowed.schedule() }

        // Stow (for now like this coz i dont wanna break anything
        button(10).changeOn(Superstructure.kStowed)

        button(9).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position + 1.inch })
        button(11).changeOn(ClosedLoopElevatorMove { Elevator.currentState.position - 1.inch })

        // that one passthrough preset that doesnt snap back to normal
//            button(4).changeOn(Superstructure.kBackHatchFromLoadingStation)

        // hatches
        lessThanAxisButton(1).change(IntakeHatchCommand(releasing = false))
        greaterThanAxisButton(1).change(IntakeHatchCommand(releasing = true))

        // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
        // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
        val cargoCommand = sequential { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake.beforeStarting { Intake.wantsOpen = true }; +IntakeCargoCommand(releasing = false) }
        lessThanAxisButton(0).changeOff { (sequential { +ClosedLoopWristMove(40.degree) ; +Superstructure.kStowed; }).schedule() }.change(cargoCommand)
        greaterThanAxisButton(0).changeOff { }.change(IntakeCargoCommand(true))

        button(4).changeOn(ClimbSubsystem.prepMove).changeOn { isClimbing = true; wantsHab3Mode = false }
        state({ isClimbing && !wantsHab3Mode }) {
            button(12).changeOn(ClimbSubsystem.hab2ClimbCommand)
        }
        state({ isClimbing && wantsHab3Mode }) {
            button(12).changeOn(ClimbSubsystem.hab3ClimbCommand)
        }

    }

    override fun update() {
        driverFalconXbox.update()
        operatorFalconHID.update()
//        auxFalconXbox.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }

private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
//        Robot.activateEmergency()
        val command = object: FalconCommand(Superstructure, DriveSubsystem, Elevator, Proximal, Wrist, Intake) {
            override fun execute() {
                Superstructure.setNeutral()
                Elevator.setNeutral()
                Proximal.setNeutral()
                Wrist.setNeutral()
                Intake.setNeutral()
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