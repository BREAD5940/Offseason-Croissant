@file:Suppress("unused")

package org.team5940.pantry.lib.statespace

import kotlin.math.PI
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt

object Newton : SIKey

val Number.revolutionsPerMinute get() = SIUnit<Frac<Radian, Second>>(toDouble() / 60.0 * 2.0 * PI)
val Number.newtonMeters get() = SIUnit<Mult<Newton, Meter>>(toDouble())

/**
 * Holds the constants for a DC brushed motor.
 * Keyword arguments:
 * nominal_voltage -- voltage at which the motor constants were measured
 * stall_torque -- current draw when stalled in Newton-meters
 * stall_current  -- current draw when stalled in Amps
 * free_current -- current draw under no load in Amps
 * free_speed -- angular velocity under no load in Rad/Sec
 */
@Suppress("MemberVisibilityCanBePrivate")
open class DCBrushedMotor(
    val nominalVoltage: SIUnit<Volt>,
    val stallTorque: SIUnit<Mult<Newton, Meter>>,
    val stallCurrent: SIUnit<Ampere>,
    val freeCurrent: SIUnit<Ampere>,
    val freeSpeed: SIUnit<Frac<Radian, Second>>
) {

    /** Resistance of motor */
    val R = SIUnit<Frac<Volt, Ampere>>(nominalVoltage.value / stallCurrent.value)

    /** Motor velocity constant */
    val kV = SIUnit<Frac<Radian, Mult<Second, Volt>>>(freeSpeed.value / (nominalVoltage.value - R.value * freeCurrent.value))

    /** Torque constant */
    val kT = SIUnit<Frac<Newton, Ampere>>(stallTorque.value / stallCurrent.value)
}

/**
 * A DCBrushedGearbox which holds a [numMotors] of [motor]s, with a [gearing] as output over input
 */
class DCBrushedGearbox(motor: DCBrushedMotor, numMotors: Int, gearing: Double) : DCBrushedMotor(
        motor.nominalVoltage, motor.stallTorque * gearing,
        motor.stallCurrent, motor.freeCurrent, motor.freeSpeed / gearing
)

fun gearbox(motor: DCBrushedMotor, numMotors: Int) = DCBrushedMotor(
        motor.nominalVoltage, motor.stallTorque * numMotors,
        motor.stallCurrent, motor.freeCurrent, motor.freeSpeed)

fun modelCIM(nominalVoltage: SIUnit<Volt>) = DCBrushedMotor(
        nominalVoltage, stallTorque = 2.42.newtonMeters, stallCurrent = 133.amps,
        freeCurrent = 2.7.amps, freeSpeed = 5310.revolutionsPerMinute
)

fun modelMiniCIM(nominalVoltage: SIUnit<Volt>) = DCBrushedMotor(
        nominalVoltage, stallTorque = 1.41.newtonMeters, stallCurrent = 89.amps,
        freeCurrent = 3.0.amps, freeSpeed = 5840.revolutionsPerMinute
)

fun modelNEO(nominalVoltage: SIUnit<Volt>) = DCBrushedMotor(
        nominalVoltage, stallTorque = 2.6.newtonMeters, stallCurrent = 150.amps,
        freeCurrent = 1.8.amps, freeSpeed = 5676.revolutionsPerMinute
)
