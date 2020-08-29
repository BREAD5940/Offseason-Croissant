package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.ManualDriveCommand
import frc.robot.subsystems.superstructure.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import org.team5940.pantry.lib.WantedState
import java.awt.Color
import kotlin.math.absoluteValue
import kotlin.math.sign

class RetractStiltCommand: FalconCommand(ClimbSubsystem) {

    override fun initialize() {
        ClimbSubsystem.stiltMotor.controller.setOutputRange(-0.5, 0.5)
        ClimbSubsystem.stiltMotor.setPosition(25.inch)
        Controls.isClimbing = true
        LEDs.wantedState = LEDs.State.Blink(0.15.second, Color(130, 24, 30))
    }

    override fun end(i: Boolean) {
        ClimbSubsystem.stiltMotor.controller.setOutputRange(-1.0, 1.0)
        Controls.isClimbing = true
        LEDs.wantedState = LEDs.State.Default
    }

    override fun isFinished() = ClimbSubsystem.stiltMotor.encoder.position > 23.inch
}

class ClimbWithElevatorRetracted: FalconCommand(DriveSubsystem, Elevator, Proximal, Wrist, Superstructure) {

    override fun initialize() {
        Elevator.wantedState = WantedState.Position(24.inch)
        Proximal.wantedState = WantedState.Position((-20).degree)
        Proximal.setMotionMagicMode()
        Controls.isClimbing = true
        LEDs.wantedState = LEDs.State.Blink(0.15.second, Color(130, 24, 30))
    }

    override fun end(i: Boolean) {
        Proximal.setMotionMagicMode()
    }

    override fun execute() {
        var s3nd = yeetForwardSource()
        if (s3nd < -0.1) s3nd = -0.2
        ClimbSubsystem.intakeWheels.setDutyCycle(s3nd)
        DriveSubsystem.lowGear = true
        DriveSubsystem.setPercent(s3nd / 5.0, s3nd / 5.0)
    }

    override fun isFinished() = Elevator.isWithTolerance(2.inch) && Proximal.isWithTolerance(6.degree)

}

val yeetForwardSource by lazy {
    { val toRet = Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kRight) - Controls
            .driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft)
//                    println("speed $toRet")
        val compensated = toRet * -1.0
        val deadbanded = ((compensated.absoluteValue - ManualDriveCommand.kDeadband / 1.8) / (1.0 - ManualDriveCommand.kDeadband / 1.8)) * compensated.sign

        // throttle curve should be between [-1. 1] or so.
        ManualDriveCommand.kisscalc(deadbanded, 0.77, 0.0, 0.00116)
    }
}