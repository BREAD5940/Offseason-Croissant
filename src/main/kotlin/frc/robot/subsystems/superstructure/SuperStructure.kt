 package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports
import kotlin.math.roundToInt

typealias Length = SIUnit<Meter>

object Superstructure : FalconSubsystem(), EmergencyHandleable, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    val kStowed
        get() = everythingMoveTo(30.25.inch + 1.2.inch, (-70).degree, 40.degree)
    val kMatchStartToStowed get() = sequential {
        +parallel {
            +ClosedLoopProximalMove((-70).degree)
            +ClosedLoopWristMove(40.degree)
        }
        +ClosedLoopElevatorMove(30.25.inch)
        +kStowed
    }
    val kBackHatchFromLoadingStation get() = SyncedMove.frontToBack
    val kHatchLow get() = everythingMoveTo(19.inch, 0.degree, 4.degree)
    val kHatchMid get() = everythingMoveTo(43.inch, 0.degree, 4.degree)
    val kHatchHigh get() = everythingMoveTo(67.inch, 0.degree, 4.degree)

    val kCargoIntake get() = everythingMoveTo(25.0.inch, (-44).degree, (-20).degree)
    val kCargoShip get() = everythingMoveTo(47.5.inch, (-5).degree, (-50).degree)
//    val kCargoShip get() = everythingMoveTo(30.25.inch + 1.2.inch + 24.inch, (-70).degree, 0.degree)

    val kCargoLow get() = everythingMoveTo(20.5.inch, 6.degree, 16.degree)
    val kCargoMid get() = everythingMoveTo(45.inch, 6.degree, 6.degree)
    val kCargoHigh get() = everythingMoveTo(64.5.inch, 7.degree, 30.degree)

    val kStraightDown get() = sequential {
        +kHatchMid
        +ClosedLoopElevatorMove(35.5.inch)
        +parallel {
            +ClosedLoopProximalMove((-100).degree)
            +ClosedLoopWristMove(-50.degree)
        }
//        +kStowed
    }

    fun everythingMoveTo(elevator: Length, proximal: SIUnit<Radian>, wrist: SIUnit<Radian>) = everythingMoveTo(State.Position(elevator, proximal, wrist))

    fun everythingMoveTo(goalState: SuperstructureState) = SuperstructurePlanner.everythingMoveTo(goalState)

    fun getUnDumbWrist(dumbWrist: SIUnit<Radian>, relevantProx: SIUnit<Radian>) =
            dumbWrist.plus(relevantProx.div(2))

    fun getUnDumbWrist(dumbWrist: Double, relevantProx: Double) =
            dumbWrist.plus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: SIUnit<Radian>, relevantProx: SIUnit<Radian>) =
            smartWrist.minus(relevantProx.div(2))

    fun getDumbWrist(smartWrist: Double, relevantProx: Double) =
            smartWrist - (relevantProx / 2)

    override fun activateEmergency() {
        Elevator.activateEmergency()
        Proximal.activateEmergency()
        Wrist.activateEmergency() }

    override fun recoverFromEmergency() {
        Elevator.recoverFromEmergency()
        Proximal.recoverFromEmergency()
        Wrist.recoverFromEmergency() }

    override fun setNeutral() {
        Elevator.wantedState = WantedState.Nothing
        Proximal.wantedState = WantedState.Nothing
        Wrist.wantedState = WantedState.Nothing

        Elevator.setNeutral()
        Proximal.setNeutral()
        Wrist.setNeutral() }

    private val currentStateMutex = Object()
    var currentState = SuperstructureState()
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    override fun updateState() {
        // update the states of our components

        Wrist.updateState()

        // use these updated states to build our current state
        val newState = State.Position(
            Elevator.updateState().position,
            Proximal.updateState().position,
            Wrist.currentState.position
//            wristUnDumb = false
        )

        currentState = newState
    }

    override fun useState() {
        Elevator.useState()
        Proximal.useState()
        Wrist.useState()
    }

    val zero = ZeroSuperStructureRoutine()
    override fun lateInit() {
        Proximal.zero()
        Wrist.zero()

        SmartDashboard.putData(zero)
        zero.schedule()

        SmartDashboard.putData(this)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addStringProperty("cState", { currentState.asString() }, { })
        builder.addDoubleProperty("cProxTranslation", { currentState.proximalTranslation().y.inch }, { })
        super.initSendable(builder)
    }

    sealed class State {

        object Nothing : State()

        data class Position(
            val elevator: SIUnit<Meter>,
            val proximal: SIUnit<Radian>,
            val wrist: SIUnit<Radian>,
            val isPassedThrough: Boolean = proximal < (-135).degree,
            val isWristUnDumb: Boolean = false
        ) : State() {

            constructor() : this(20.inch, (-90).degree, (-45).degree) // semi-sane numbers?

            fun proximalTranslation() =
                    Translation2d(kProximalLen, proximal.toRotation2d()) + Translation2d(0.meter, elevator)

            fun dumbState() = if (!isWristUnDumb) this else Position(elevator, proximal, getDumbWrist(wrist, proximal))
            fun trueState() = if (isWristUnDumb) this else Position(elevator, proximal, getUnDumbWrist(wrist, proximal))

            fun asString(): String {
                return "Elevator [${(elevator.inch).roundToInt()}\"] proximal [${proximal.degree.roundToInt()}deg] wrist [${wrist.degree.roundToInt()}deg]"
            }

            override fun toString(): String = asString()
        }

        abstract class CustomState : State() {
            abstract fun useState()
        }
    }
}

typealias SuperstructureState = Superstructure.State.Position
