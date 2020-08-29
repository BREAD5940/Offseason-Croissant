package org.team5940.pantry.lib

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconCTRE
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem

/**
 * A MultiMotorTransmission contains multiple motors.
 * It is paramatrized with a SIUnit of type [T] (ex. Length),
 * and a master of type [M] with a model of type [T]
 */
abstract class MultiMotorTransmission<T : SIKey, M : FalconMotor<T>> : FalconMotor<T>,
        SensorlessCompatibleSubsystem, ConcurrentlyUpdatingJoint<T> {

    abstract val master: M

    protected open val followers: List<FalconMotor<*>>? = null

    override val encoder by lazy { master.encoder }
//        @Synchronized get
    override var motionProfileAcceleration
        get() = master.motionProfileAcceleration
        set(value) { master.motionProfileAcceleration = value }
    override var motionProfileCruiseVelocity: SIUnit<Velocity<T>>
        get() = master.motionProfileCruiseVelocity
        set(value) { master.motionProfileCruiseVelocity = value }
    override var outputInverted
        get() = master.outputInverted
        set(value) {
            master.outputInverted = value
            followers?.forEach {
                it.outputInverted = value
            }
        }
    override var useMotionProfileForPosition
        get() = master.useMotionProfileForPosition
        set(value) { master.useMotionProfileForPosition = value }
    override var voltageCompSaturation
        get() = master.voltageCompSaturation
        set(value) { master.voltageCompSaturation = value }
    override val voltageOutput
        get() = master.voltageOutput
    override var brakeMode
        get() = master.brakeMode
        set(value) { master.brakeMode = value }
    override val drawnCurrent: SIUnit<Ampere>
        get() {
            var ret = master.drawnCurrent
            followers?.forEach {
                ret += it.drawnCurrent
            }
            return ret // / (1 + (followers?.size ?: 0))
        }

    override fun follow(motor: FalconMotor<*>): Boolean {
        var result = true

        result = result and master.follow(motor)

        val followers_ = followers

        followers_?.forEach {
            result = result and it.follow(motor)
        }

        return result
    }

    override fun setDutyCycle(dutyCycle: Double, arbitraryFeedForward: SIUnit<Volt>) =
            master.setDutyCycle(dutyCycle, arbitraryFeedForward)

    override fun setNeutral() {
        master.setNeutral()
    }

    override fun setPosition(position: SIUnit<T>, arbitraryFeedForward: SIUnit<Volt>) =
            master.setPosition(position, arbitraryFeedForward)

    override fun setVelocity(velocity: SIUnit<Velocity<T>>, arbitraryFeedForward: SIUnit<Volt>) =
            master.setVelocity(velocity, arbitraryFeedForward)

    override fun setVoltage(voltage: SIUnit<Volt>, arbitraryFeedForward: SIUnit<Volt>) =
        master.setVoltage(voltage, arbitraryFeedForward)

    fun zeroClosedLoopGains() {
        val list = followers
        val motorList: List<FalconMotor<*>> = if (list == null) listOf(master) else list + master

        motorList.forEach { motor ->

            when (motor) {
                is FalconCTRE<*> -> { motor.motorController.run {
                    config_kP(0, 0.0, 0)
                    config_kI(0, 0.0, 0)
                    config_kD(0, 0.0, 0)
                    configClosedLoopPeakOutput(0, 0.0, 0)
                } }
                is FalconMAX<*> -> { motor.controller.p = 0.0; motor.controller.i = 0.0; motor.controller.d = 0.0
                    motor.controller.setOutputRange(0.0, 0.0) }
            }
        }
    }

    abstract fun setClosedLoopGains()

    fun lateInit() {
        followers?.forEach { it.follow(master) }
    }

    val outputCurrent: Double
        get() {
            val master_: FalconMotor<T> = master

            return when (master_) {
                is FalconCTRE -> {
                    when (master_.motorController) {
                        is IMotorControllerEnhanced -> {
                            (master_.motorController as IMotorControllerEnhanced).outputCurrent
                        } else -> 0.0
                    }
                }
                is FalconMAX -> {
                    master_.canSparkMax.outputCurrent
                }
                else -> 0.0
            }
        }

    fun FalconMotor<*>.setClosedLoopGains(p: Double, d: Double, ff: Double = 0.0) {
        when (this) {
            is FalconCTRE -> { this.motorController.run {
                config_kP(0, p, 0)
                config_kI(0, 0.0, 0)
                config_kD(0, d, 0)
                config_kF(0, ff, 0)
            } }
            is FalconMAX -> { this.controller.p = p; this.controller.i = 0.0; this.controller.d = d; this.controller.ff = ff }
        }
    }

    fun setClosedLoopGains(p: Double, d: Double, ff: Double = 0.0) = master.setClosedLoopGains(p, d, ff)

    override fun disableClosedLoopControl() {
        zeroClosedLoopGains()
        setNeutral()
    }

    override fun enableClosedLoopControl() {
        setClosedLoopGains()
    }

    data class State<T : SIKey>(
        val position: SIUnit<T>, // the position in [T] units
        val velocity: SIUnit<Velocity<T>> = SIUnit<T>(0.0).velocity,
        val acceleration: SIUnit<Acceleration<T>> = SIUnit<T>(0.0).acceleration
    )

    internal val currentStateMutex = Object()
    var currentState = State(SIUnit<T>(0.0))
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    private var lastUpdateTime = Timer.getFPGATimestamp()

    override fun updateState(): State<T> {
        val now = Timer.getFPGATimestamp()
        val position = encoder.position
        val lastState = currentState
        val velocity = encoder.velocity
        val acceleration = (velocity - lastState.velocity) / (now - lastUpdateTime) / 1.second

        // add the observation to the current state channel
        val newState = State(position, velocity, acceleration)

        lastUpdateTime = now

        currentState = newState

        return newState
    }

    /**
     * S3nd the wanted demand to the motor RIGHT NOW
     * @param wantedState the wanted state
     * @param arbitraryFeedForward the arbitraryFeedForward in Volts
     */
    fun s3ndState(wantedState: WantedState?, arbitraryFeedForward: SIUnit<Volt>) {
        if (wantedState == null) return

        when (wantedState) {
            is WantedState.Nothing -> setNeutral()
            is WantedState.Position<*> -> {
                val ff = if (wantedState.ignoreDefaultFF) wantedState.feedForward else arbitraryFeedForward
                setPosition(SIUnit(wantedState.targetPosition.value), ff)
            }
            is WantedState.Velocity<*> -> setVelocity(SIUnit(wantedState.targetVelocity.value), arbitraryFeedForward)
            is WantedState.Voltage -> setVoltage(wantedState.output, arbitraryFeedForward)
            is WantedState.CustomState -> wantedState.useState(wantedState)
        }
    }
}

private operator fun <T : SIKey> SIUnit<Velocity<T>>.div(second: SIUnit<Second>) = SIUnit<Frac<Velocity<T>, Second>>(this.value / second.value)
