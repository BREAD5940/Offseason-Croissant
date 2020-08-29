package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice
import com.ctre.phoenix.motorcontrol.RemoteSensorSource
import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kProximalCos
import frc.robot.Constants.SuperStructureConstants.kProximalStatic
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.Meter
import kotlin.math.cos
import kotlin.math.withSign
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports

object Proximal : ConcurrentFalconJoint<Radian, FalconSRX<Radian>>() {

    fun resetPosition(position: SIUnit<Radian>) {
        val ticks = (position).toNativeUnitPosition(Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)
        canifier.setQuadraturePosition(ticks.value.toInt(), 0)
    }

    val activeTrajectoryPosition: SIUnit<Radian>
        get() {
            val nativeTrajectoryPos = motor.master.talonSRX.activeTrajectoryPosition.nativeUnits
            return Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL.fromNativeUnitPosition(nativeTrajectoryPos)
        }

    fun configureThrust(thrustVelocity: SIUnit<AngularVelocity>) {
        motor.motionProfileCruiseVelocity = thrustVelocity
    }

    val canifier = CANifier(34)
    val absoluteEncoder = canifier.asPWMSource(1131.0 to (-90).degrees, 677.0 to 0.degrees,
            CANifier.PWMChannel.PWMChannel0)

    fun zero() = resetPosition(absoluteEncoder())

    override fun periodic() {
//        val prox = doubleArrayOf(0.0, 0.0)
//        val wrist = doubleArrayOf(0.0, 0.0)
//        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, prox)
//        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel1, wrist)

//        println("${prox[0]}, ${wrist[0]}") // 541.3, 19977 to 2394, 19977
//        println("${prox[0]}/${absoluteEncoder().degree}, ${wrist[0]}/${Wrist.absoluteEncoder().degree}")
//        println(Wrist.motor.encoder.position.degree)
    }

    override val motor = object : MultiMotorTransmission<Radian, FalconSRX<Radian>>() {

        override val master: FalconSRX<Radian> = FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[0],
                Ports.SuperStructurePorts.ProximalPorts.ROTATION_MODEL)

        override val followers: List<FalconSRX<*>> = listOf(
                FalconSRX(Ports.SuperStructurePorts.ProximalPorts.TALON_PORTS[1], DefaultNativeUnitModel))

        init {
            if (Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION.size < followers.size)
                throw ArrayIndexOutOfBoundsException("Follower inversion list size contains less indices than the number of followers!")

            master.talonSRX.configRemoteFeedbackFilter(canifier.deviceID, RemoteSensorSource.CANifier_Quadrature, 0, 100)
            master.talonSRX.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 100)

            master.outputInverted = Ports.SuperStructurePorts.ProximalPorts.TALON_INVERTED
            master.talonSRX.configSelectedFeedbackSensor(Ports.SuperStructurePorts.ProximalPorts.SENSOR)
            master.talonSRX.setSensorPhase(Ports.SuperStructurePorts.ProximalPorts.TALON_SENSOR_PHASE)
            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
            master.talonSRX.configPeakOutputForward(1.0)
            master.talonSRX.configPeakOutputReverse(-1.0)

            followers.forEachIndexed {
                index, followerMotor ->

                followerMotor.follow(master)
                followers[index].talonSRX.setInverted(Ports.SuperStructurePorts.ProximalPorts.FOLLOWER_INVERSION[index])

                master.talonSRX.configClosedLoopPeakOutput(0, 1.0)
                master.talonSRX.configPeakOutputForward(1.0)
                master.talonSRX.configPeakOutputReverse(-1.0)
            }
            setClosedLoopGains()

            master.talonSRX.configForwardSoftLimitEnable(false)
            master.talonSRX.configReverseSoftLimitEnable(false)
        }

        override fun setClosedLoopGains() {
            master.useMotionProfileForPosition = true
            // TODO use FalconSRX properties for velocities and accelerations
            master.talonSRX.configMotionCruiseVelocity((1785.0 * Constants.SuperStructureConstants.kJointSpeedMultiplier).toInt()) // about 3500 theoretical max
            master.talonSRX.configMotionAcceleration(5000)
            master.talonSRX.configMotionSCurveStrength(0)

            master.talonSRX.configClosedLoopPeakOutput(0, 1.0)

            master.setClosedLoopGains(
                    0.5 * 1.2, 6.0, ff = 0.45 // used to be p = 0.85, got oscolation
            )
        }

        override var softLimitForward: SIUnit<Radian>
            get() = master.softLimitForward
            set(value) {
                master.softLimitForward = value
            }

        override var softLimitReverse: SIUnit<Radian>
            get() = master.softLimitReverse
            set(value) {
                master.softLimitReverse = value
            }
    }

    fun setClimbPositionMode() = motor.run {
        setClosedLoopGains(3.0, 0.0, 0.0)
        useMotionProfileForPosition = false
    }
    fun setMotionMagicMode() = motor.run {
        setClosedLoopGains()
        useMotionProfileForPosition = true
    }

    override fun calculateFeedForward(currentState: MultiMotorTransmission.State<Radian>) =
            kProximalStatic.withSign(currentState.velocity.value) + (kProximalCos * cos(currentState.position).value)
}

fun <K : SIKey> SIUnit<K>.withSign(sign: Number) = SIUnit<K>(this.value.withSign(sign.toDouble()))

fun <K : SIKey> cos(x: SIUnit<K>) = SIUnit<K>(cos(x.value))
