package org.team5940.pantry.lib

import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volt

sealed class WantedState {
    object Nothing : WantedState() {
        override fun toString(): String = "Nothing"
    }

    class Position<T : SIKey>(val targetPosition: SIUnit<T>, val feedForward: SIUnit<Volt> = 0.volt, val ignoreDefaultFF: Boolean = false) : WantedState() {
        operator fun plus(delta: SIUnit<T>) = Position(targetPosition + delta)

        fun coerceIn(range: ClosedRange<SIUnit<T>>) = Position(
                targetPosition.coerceIn(range)
        )
        override fun toString(): String = "Position $targetPosition"
    }

    class Velocity<K : SIKey>(val targetVelocity: SIUnit<org.ghrobotics.lib.mathematics.units.derived.Velocity<K>>) : WantedState() {
        override fun toString(): String = "Velocity $targetVelocity"
    }

    class Voltage(val output: SIUnit<Volt>) : WantedState() {
        override fun toString() = "Voltage $output"
    }

    abstract class CustomState : WantedState() {
        abstract fun useState(wantedState: WantedState)
        override fun toString(): String = "CustomState"
    }
}

typealias LinearJointState = WantedState
