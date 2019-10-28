package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.climb.ClimbSubsystem
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import java.awt.Color

class ZeroSuperStructureRoutine(private val mZeroHeight: Length = kZeroHeight) : FalconCommand(Superstructure,
        Elevator, Proximal, Wrist) {

    private var mCurrentState: ZeroingState = ZeroingState.IDLE

    private var isDone = false

    override fun runsWhenDisabled(): Boolean {
        return true
    }

    private enum class ZeroingState {
        IDLE, WAITING_FOR_TRIGGER, ZEROED
    }

    // Called just before this Command runs the first time
    override fun initialize() {
        mCurrentState = ZeroingState.IDLE
        SmartDashboard.putBoolean("Elevator zeroed", false)
        LEDs.wantedState = LEDs.State.Solid(LEDs.PURPLE)
    }

    // Called repeatedly when this Command is scheduled to run
    override fun execute() {

        val limitTriggered = Elevator.limitSwitchTriggered

        SmartDashboard.putString("Zeroing state", mCurrentState.name)
        SmartDashboard.putBoolean("Elevator limit switch", limitTriggered)

        val proxPos = Proximal.absoluteEncoder()
        val wristPos = Wrist.absoluteEncoder()

        SmartDashboard.putNumber("proxPos", proxPos.degree)
        SmartDashboard.putNumber("wristPos", wristPos.degree)

        if (mCurrentState == ZeroingState.IDLE) {
            if (!limitTriggered) {
                mCurrentState = ZeroingState.WAITING_FOR_TRIGGER
            }
        } else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
            if (limitTriggered) {
                observeElevatorZero()
                mCurrentState = ZeroingState.ZEROED
                isDone = true
            }
        }
    }

    private fun observeElevatorZero() {

        Elevator.motor.master.talonSRX.set(ControlMode.PercentOutput, 0.0)
        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()

        SmartDashboard.putBoolean("Elevator zeroed", true)
        SmartDashboard.putBoolean("Proximal zeroed", true)
        SmartDashboard.putBoolean("Wrist zeroed", true)

        val proxPos = Proximal.absoluteEncoder() - 6.degree //  (-90).degree
        val wristPos = Wrist.absoluteEncoder() - 3.degree
        Proximal.resetPosition(proxPos)
        Wrist.resetPosition(wristPos)
        ClimbSubsystem.zero()
        Elevator.motor.encoder.resetPositionRaw(Elevator.motor.master.model.toNativeUnitPosition(mZeroHeight))
    }

    // Make this return true when this Command no longer needs to run execute()
    override fun isFinished(): Boolean {
        if (mCurrentState == ZeroingState.ZEROED) println("We're zeroed so we're done")
        val shouldEnd = isDone // || Robot.isEnabled
        return shouldEnd
    }

    // Called once after isFinished returns true
    override fun end(interrupted: Boolean) {
        println("ENDING ${javaClass.simpleName}")
//        Elevator.elevatorZeroed = !interrupted
        SmartDashboard.putString("Zeroing state", mCurrentState.name)
        LEDs.wantedState = LEDs.State.Blink(0.125.second, Color.GREEN)
        GlobalScope.launch {
            delay(1500)
            LEDs.wantedState = LEDs.State.Off
            delay(250)
            LEDs.wantedState = LEDs.State.Default
        }
    }

    companion object {
        private val kZeroHeight = 35.inch // 21.5.inch, delta is 11.5in
    }
}
