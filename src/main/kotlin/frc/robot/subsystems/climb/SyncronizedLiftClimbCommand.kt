package frc.robot.subsystems.climb

import frc.robot.subsystems.superstructure.Elevator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inches
import org.team5940.pantry.lib.WantedState

class SyncronizedLiftClimbCommand : FalconCommand(ClimbSubsystem, Elevator) {

    var elevatorInPosition = false
    var stiltsInPosition = false

    override fun initialize() {
        ClimbSubsystem.stiltMotor.canSparkMax.setSmartCurrentLimit(5) // amps?
    }

    override fun execute() {
        // let's do voltage PID on the NEOOOOOOOOO
//        val elevatorPosition = WantedState.Position(Elevator.currentState.position)
        if (Elevator.currentState.position < 10.inches) elevatorInPosition = true
        if (!elevatorInPosition) Elevator.wantedState = WantedState.Voltage((-5).volt) else
            Elevator.wantedState = WantedState.Position(10.inches)
//        ClimbSubsystem.wantedState = if(!stiltsInPosition) WantedState.Voltage((-5).volt) else WantedState.Position(10.inch)
        if (!stiltsInPosition) ClimbSubsystem.stiltMotor.setVoltage((-5).volt) else ClimbSubsystem.stiltMotor.setPosition(15.inches)
    }

    override fun isFinished(): Boolean = elevatorInPosition and stiltsInPosition
}

// class SketchyTest : FalconCommand(ClimbSubsystem, Superstructure, Elevator, Proximal, Wrist) {
//    override fun isFinished() = false
//
//    val elevatorVoltage by lazy { Controls.auxFalconXbox.getY(GenericHID.Hand.kLeft) }
//    val stiltVoltage by lazy { Controls.auxFalconXbox.getY(GenericHID.Hand.kRight) }
//
//    override fun execute() {
//        ClimbSubsystem.intakeWheels.setDutyCycle(0.2)
//        if(Elevator.currentState.position < 15.inch) {
//            Elevator.wantedState = WantedState.Position(15.inch)
//        } else {
//            Elevator.wantedState = WantedState.Voltage(SIUnit(12.0 * elevatorVoltage()))
//        }
//
//        if(ClimbSubsystem.currentState.position < 15.inch) {
// //            ClimbSubsystem.wantedState = WantedState.Position(15.inch)
//            ClimbSubsystem.stiltMotor.setPosition(15.inch)
//        } else {
// //            ClimbSubsystem.wantedState = WantedState.Voltage(SIUnit(12.0 * stiltVoltage()))
//            ClimbSubsystem.stiltMotor.setVoltage(12.0.volt * stiltVoltage())
//        }
//        println("ELEVATOR VOLTAGE ${elevatorVoltage.invoke() * 12.0} STILTS ${stiltVoltage.invoke() * 12.0}")
//
//    }
// }
