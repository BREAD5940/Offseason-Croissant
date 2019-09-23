package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.frc2.command.Command
import edu.wpi.first.wpilibj.frc2.command.ConditionalCommand
import edu.wpi.first.wpilibj.frc2.command.InstantCommand
import edu.wpi.first.wpilibj.frc2.command.PrintCommand
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.climb.SketchyTest
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeCargoCommand
import frc.robot.subsystems.intake.IntakeHatchCommand
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructurePlanner
import frc.robot.subsystems.superstructure.ZeroSuperStructureRoutine
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports
import org.team5940.pantry.lib.Updatable
import org.team5940.pantry.lib.WantedState
import java.util.function.BooleanSupplier

object Controls : Updatable {

    var isClimbing = false
        private set

    private val zero = ZeroSuperStructureRoutine()

    val driverFalconXbox = xboxController(0) {
        registerEmergencyMode()

        button(kB).changeOn { isClimbing = true }
        button(kX).changeOn { isClimbing = false }

        state({ !isClimbing }) {
            // Vision align
            triggerAxisButton(GenericHID.Hand.kRight).change(
                    ConditionalCommand(VisionDriveCommand(true), VisionDriveCommand(false),
                            BooleanSupplier { !Superstructure.currentState.isPassedThrough }))

            // Shifting
            button(kBumperLeft).changeOn { DriveSubsystem.lowGear = true }.changeOff { DriveSubsystem.lowGear = false }
//            button(kA).changeOn { ZeroSuperStructureRoutine().schedule() }
            button(kA).changeOn(sequential {
                +SuperstructurePlanner.everythingMoveTo(35.inch, 0.degree, 0.degree) // TODO check preset
                +SuperstructurePlanner.everythingMoveTo(25.inch, (-5).degree, 90.degree) // TODO check preset
            })
        }
        state({ isClimbing }) {
            button(kA).changeOn(SketchyTest())
        }
    }

    private val operatorJoy = Joystick(5)
    val operatorFalconHID = operatorJoy.mapControls {

        state({ !isClimbing }) {

            // elevator jogging
            button(9).changeOn { Elevator.elevatorOffset += 0.3.inch }
            button(11).changeOn { Elevator.elevatorOffset -= 0.3.inch }

            // cargo presets
            button(12).changeOn(Superstructure.kCargoIntake.andThen { Intake.wantsOpen = true }) // .changeOff { Superstructure.kStowed.schedule() }
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

            // that one passthrough preset that doesnt snap back to normal
//            button(4).changeOn(Superstructure.kBackHatchFromLoadingStation)

            // hatches
            lessThanAxisButton(1).change(IntakeHatchCommand(releasing = false))
            greaterThanAxisButton(1).change(IntakeHatchCommand(releasing = true))

            // cargo -- intake is a bit tricky, it'll go to the intake preset automatically
            // the lessThanAxisButton represents "intaking", and the greaterThanAxisButton represents "outtaking"
            val cargoCommand = sequential { +PrintCommand("running cargoCommand"); +Superstructure.kCargoIntake; +IntakeCargoCommand(releasing = false) }
            lessThanAxisButton(0).changeOff { Superstructure.kStowed.schedule() }.change(cargoCommand)
            greaterThanAxisButton(0).changeOff { Superstructure.kStowed.schedule() }.change(IntakeCargoCommand(true))
        }
    }

    override fun update() {
        driverFalconXbox.update()
        operatorFalconHID.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }

private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn {
        Robot.activateEmergency()
    }
    button(kStart).changeOn {
        Robot.recoverFromEmergency()
    }
}