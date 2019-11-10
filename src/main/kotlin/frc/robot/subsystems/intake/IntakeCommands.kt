@file:Suppress("unused", "MemberVisibilityCanBePrivate")

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.volt

val closeIntake = InstantCommand(Runnable { IntakeSubsystem.wantsOpen = false })
val openIntake = InstantCommand(Runnable { IntakeSubsystem.wantsOpen = true })

class IntakeHatchCommand(val releasing: Boolean, val shouldMutateArms: Boolean = true) : FalconCommand(IntakeSubsystem) {

    override fun initialize() {
//        println("intaking hatch command")
        IntakeSubsystem.hatchMotorOutput = 12.volt * (if (releasing) -1 else 1)
        IntakeSubsystem.cargoMotorOutput = 0.volt
        if (shouldMutateArms) IntakeSubsystem.wantsOpen = false // we want to be closed regardless (can't outtake or intake with it open
    }

    override fun end(interrupted: Boolean) {
        if (shouldMutateArms) IntakeSubsystem.wantsOpen = !releasing // we want the intake to be open if we were just intaking, and closed if we were just outtaking
        IntakeSubsystem.hatchMotorOutput = 0.volt
        IntakeSubsystem.cargoMotorOutput = 0.volt
    }
}

class IntakeCargoCommand(val releasing: Boolean) : FalconCommand(IntakeSubsystem) {

//    var wasOpen: Boolean = false

    override fun execute() {
        println("${if (releasing) "releasing" else "intaking"} cargo command!")
//        wasOpen = IntakeSubsystem.wantsOpen
        IntakeSubsystem.wantsOpen = !releasing

        IntakeSubsystem.hatchMotorOutput = 12.volt * (if (releasing) 1 else -1)
        IntakeSubsystem.cargoMotorOutput = 12.volt * (if (!releasing) 1 else -1)

        super.initialize()
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.wantsOpen = false
        IntakeSubsystem.cargoMotorOutput = 3.volt
        IntakeSubsystem.hatchMotorOutput = 3.volt
        GlobalScope.launch {
            delay(500)
            IntakeSubsystem.cargoMotorOutput = 0.volt
            IntakeSubsystem.hatchMotorOutput = 0.volt
        }
        super.end(interrupted)
    }
}

// class IntakeTeleopCommand : FalconCommand(IntakeSubsystem) {
//
//    override fun execute() {
//        val cargoSpeed = -cargoSource()
//        val hatchSpeed = -hatchSource()
//
//        if (abs(cargoSpeed) > 0.2) {
//            IntakeSubsystem.hatchMotorOutput = (-12).volt * cargoSpeed
//            IntakeSubsystem.cargoMotorOutput = 12.volt * cargoSpeed
//        } else {
//            IntakeSubsystem.hatchMotorOutput = 12.volt * hatchSpeed
//            IntakeSubsystem.cargoMotorOutput = 0.volt
//        }
//    }
//
//    override fun end(interrupted: Boolean) {
//        IntakeSubsystem.hatchMotorOutput = 0.volt
//        IntakeSubsystem.cargoMotorOutput = 0.volt
//    }
//
//    companion object {
//        val cargoSource by lazy { Controls.operatorFalconHID.getRawAxis(0) }
//        val hatchSource by lazy { Controls.operatorFalconHID.getRawAxis(1) }
//    }
// }

class IntakeCloseCommand : FalconCommand(IntakeSubsystem) {
    init {
        IntakeSubsystem.wantsOpen = false
    }
}
