package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.ControlMode

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

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
//        SmartDashboard.putData(this)
//        println("zeroing INIT")
    }

    // Called repeatedly when this Command is scheduled to run
    override fun execute() {
//        println("zeroing EXECUTE")
//        SmartDashboard.putNumber("random", Math.random())

        val limitTriggered = Elevator.limitSwitchTriggered

//        println("limitxTriggered $limitTriggered")

        SmartDashboard.putString("Zeroing state", mCurrentState.name)
        SmartDashboard.putBoolean("Elevator limit switch", limitTriggered)

        val positions = getPositions()

        SmartDashboard.putNumber("prox sensor pos", positions[0].toDouble())
        SmartDashboard.putNumber("wrist sensor pos", positions[1].toDouble())

//        if (!DriverStation.getInstance().isDisabled)
//            return

        if (mCurrentState == ZeroingState.IDLE) {
            if (!limitTriggered) {
                mCurrentState = ZeroingState.WAITING_FOR_TRIGGER
            }
        } else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
            if (limitTriggered) {
                observeElevatorZero(positions)
                mCurrentState = ZeroingState.ZEROED
                isDone = true
            }
        }
    }

    private fun observeElevatorZero(positions: List<Int>) {

        Elevator.motor.master.talonSRX.set(ControlMode.PercentOutput, 0.0)
        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral()

//        Wrist.master.talonSRX.configFeedbackNotContinuous()

        SmartDashboard.putBoolean("Elevator zeroed", true)
        SmartDashboard.putBoolean("Proximal zeroed", true)
        SmartDashboard.putBoolean("Wrist zeroed", true)

        val proximalPWM = positions[0]
        val proximalTargetPWM = 400
        val proximalPWMDelta = (proximalPWM - proximalTargetPWM) * -1 // whyyyy?
        val proxStartingPosNativeUnits = Proximal.motor.master.model.toNativeUnitPosition((-94).degree).value // .talonSRX.getTicks(RoundRotation2d.getDegree(-78))\
        Proximal.motor.master.talonSRX.selectedSensorPosition = (0.0 + proxStartingPosNativeUnits - proximalPWMDelta).toInt()

        val wristPWM = positions[1]
        val targetWristComp = 3200
        val wristPWMDelta = (wristPWM - targetWristComp).nativeUnits
        val wristStartPosNativeUnits = Wrist.motor.master.model.toNativeUnitPosition((-45).degree) // .getTicks(RoundRotation2d.getDegree(-43 + 4 - 9)) as Int

        Wrist.motor.encoder.resetPositionRaw(wristPWMDelta + wristStartPosNativeUnits)

//        Wrist.motor.master.talonSRX.selectedSensorPosition = (wristPWMDelta + wristStartPosNativeUnits).toInt()

        Elevator.motor.encoder.resetPositionRaw(Elevator.motor.master.model.toNativeUnitPosition(mZeroHeight))
    }

    fun getPositions(): List<Int> {

        val prox = (Proximal.motor.master.talonSRX.sensorCollection.pulseWidthPosition % 4096).let {
            var temp = it
            while (temp < 0) temp += 4096
            while (temp > 4096) temp -= 4096
            temp
        }
        val wrist = (Wrist.motor.master.talonSRX.sensorCollection.pulseWidthPosition % 4096).let {
            var temp = it
            while (temp < 0) temp += 4096
            while (temp > 4096) temp -= 4096
            temp
        }

        return listOf(prox, wrist)
    }

    // Make this return true when this Command no longer needs to run execute()
    override fun isFinished(): Boolean {
//        println("IS FINISHED WAS CALLED")
        if (mCurrentState == ZeroingState.ZEROED) println("We're zeroed so we're done")
//        if(Robot.lastRobotMode != FishyRobot.Mode.DISABLED) println("ds NOT disabled, returning")
//        return mCurrentState == ZeroingState.ZEROED // || Robot.lastRobotMode != FishyRobot.Mode.DISABLED
        val shouldEnd = isDone // || Robot.isEnabled
//        println("should end? $shouldEnd")
        return shouldEnd
    }

    // Called once after isFinished returns true
    override fun end(interrupted: Boolean) {
        println("ENDING ${javaClass.simpleName}")
//        Elevator.elevatorZeroed = !interrupted
        SmartDashboard.putString("Zeroing state", mCurrentState.name)
    }

    companion object {
        private val kZeroHeight = 33.inch // 21.5.inch, delta is 11.5in
    }
}
