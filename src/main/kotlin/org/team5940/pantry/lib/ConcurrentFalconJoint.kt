@file:Suppress("EXPERIMENTAL_API_USAGE")

package org.team5940.pantry.lib

import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.sync.Mutex
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import kotlin.math.abs

interface ConcurrentlyUpdatingJoint {
    fun updateState(): JointState
    fun useState() {}
}


typealias JointState = MultiMotorTransmission.State

/**
 * A joint which concurrently updates and sends demands using Channels for
 * both it's current state, a [JointState], and recieves
 * demands of type [WantedState].
 */
abstract class ConcurrentFalconJoint<T : SIUnit<T>, V : FalconMotor<T>> : ConcurrentlyUpdatingJoint,
        LoggableFalconSubsystem(), EmergencyHandleable {

    abstract val motor: MultiMotorTransmission<T, V>

    override fun activateEmergency() = motor.activateEmergency()
    override fun recoverFromEmergency() = motor.recoverFromEmergency()
    override fun setNeutral() {
        wantedState = WantedState.Nothing
        motor.setNeutral() }

    internal val wantedStateChannel = Channel<WantedState>(Channel.CONFLATED)

    open val currentState: MultiMotorTransmission.State = motor.currentState

    internal val wantedStateMutex = Object()
    /**
     * The current wantedState of the joint.
     * Setting this will both set the backing field and s3nd the new demand into the [wantedStateChannel]
     * Getting this will return the field
     *
     * Only get this from the main thread!
     */
    open var wantedState: WantedState = WantedState.Nothing
        get() = synchronized(wantedStateMutex) { field }
        set(newValue) = synchronized(wantedStateMutex) { field = newValue }

    fun isWithTolerance(tolerance: Double /* radian */): Boolean {
        val state = wantedState as? WantedState.Position ?: return false // smart cast state, return false if it's not Position

        return abs(state.targetPosition - currentState.position) < tolerance
    }

    /**
     * Calculate the arbitrary feed forward given the [currentState] in Volts
     */
    open fun calculateFeedForward(currentState: JointState) = 0.0

    open fun customizeWantedState(wantedState: WantedState) = wantedState

    override fun updateState() = motor.updateState()
    override fun useState() {

        val newState = this.wantedState
        val currentState = this.currentState

//        println("new wanted state is $newState")

        val customizedState = customizeWantedState(newState)
        val feedForward = calculateFeedForward(currentState)

        motor.s3ndState(customizedState, feedForward)
    }
}