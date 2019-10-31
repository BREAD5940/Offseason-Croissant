package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice
import com.ctre.phoenix.motorcontrol.RemoteSensorSource
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.nativeunit.toNativeUnitPosition
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.ConcurrentFalconJoint
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.WantedState
import org.team5940.pantry.lib.asPWMSource
import kotlin.math.abs

object Wrist : ConcurrentFalconJoint<Radian, FalconSRX<Radian>>() {

    fun resetPosition(position: SIUnit<Radian>) {
        val ticks = position.toNativeUnitPosition(motor.master.model)
        println("reseting position to ${position.degree} or ${ticks.value}")
        canifier.setQuadraturePosition(ticks.value.toInt() * -1, 0)
    }

    override fun periodic() {
//        zero()
//        println(motor.encoder.position.degree)
    }

    private val canifier = CANifier(35)
    val absoluteEncoder = Proximal.canifier.asPWMSource(1336.4 to (-45).degree, 2004.0 to 90.degree,
            CANifier.PWMChannel.PWMChannel1)

    fun zero() = resetPosition(absoluteEncoder())

    override val motor = object : MultiMotorTransmission<Radian, FalconSRX<Radian>>() {
        override val master: FalconSRX<Radian> = FalconSRX(Ports.SuperStructurePorts.WristPorts.TALON_PORTS,
                Ports.SuperStructurePorts.WristPorts.ROTATION_MODEL).apply { talonSRX.apply {

            var errorCode = configRemoteFeedbackFilter(canifier.deviceID, RemoteSensorSource.CANifier_Quadrature, 0, 100)

            if (errorCode != ErrorCode.OK)
                DriverStation.reportError("Could not set proximal remote sensor!!:  $errorCode", false)

            errorCode = configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 100)

            if (errorCode != ErrorCode.OK)
                DriverStation.reportError("Could not set proximal remote feedback sensor!! $errorCode", false)
        } }

        init {
            master.outputInverted = Ports.SuperStructurePorts.WristPorts.TALON_INVERTED
            master.feedbackSensor = Ports.SuperStructurePorts.WristPorts.SENSOR
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.WristPorts.TALON_SENSOR_PHASE)

            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            master.talonSRX.configPeakOutputForward(1.0)
            master.talonSRX.configPeakOutputReverse(-1.0)

            followers?.forEach {
                val motor2 = it as FalconSRX<*>
                motor2.talonSRX.configClosedLoopPeakOutput(0, 1.0)
                motor2.talonSRX.configPeakOutputForward(1.0)
                motor2.talonSRX.configPeakOutputReverse(-1.0)
            }

            setClosedLoopGains()
        }

        override fun setClosedLoopGains() = setMotionMagicGains()

        fun setMotionMagicGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((2000.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(3500)
            master.talonSRX.configMotionSCurveStrength(0)

            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)

            master.setClosedLoopGains(
                    2.0, 0.0, ff = 0.4
            )
        }
    }

    private val safeOffset = (-15).degree..15.degree
    var offset = 0.degree
        set(value) {
            field = value.coerceIn(safeOffset)
        }

    /**
     * Determine if the joint is within the [tolerance] of the current wantedState.
     * If the wantedState isn't [WantedState.Position<*>], return false.
     */
    fun isWithToleranceWithOffset(tolerance: SIUnit<Radian> /* radian */): Boolean {
        val state = wantedState as? WantedState.Position<*> ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition.value - (currentState.position.value + offset.value)) < tolerance.value
    }

    override fun customizeWantedState(wantedState: WantedState): WantedState {
        return when(wantedState) {
            is WantedState.Position<*> -> WantedState.Position(
                    SIUnit<Radian>(wantedState.targetPosition.value + offset.value),
                    wantedState.feedForward,
                    wantedState.ignoreDefaultFF)
            else -> wantedState
        }
    }

    @Suppress("UNREACHABLE_CODE")
    fun setPositionMode() = motor.run {
        setClosedLoopGains(0.5, 0.0, 0.0)
        useMotionProfileForPosition = false
    }
    fun setMotionMagicMode() = motor.run {
        setClosedLoopGains()
        useMotionProfileForPosition = true
    }
}