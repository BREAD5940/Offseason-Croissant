 package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import kotlin.math.roundToInt
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.team5940.pantry.lib.* // ktlint-disable no-wildcard-imports

typealias Length = SIUnit<Meter>

object Superstructure : FalconSubsystem(), SensorlessCompatibleSubsystem, ConcurrentlyUpdatingComponent {

    init {
        // force instantiation of subsystems
        Elevator
        Proximal
        Wrist
    }

    val kStowed
        get() = everythingMoveTo(30.25.inches + 1.2.inches - 0.75.inches, (-70).degrees, 40.degrees)

    val kPokedStowed get() = everythingMoveTo(31.45.inch - 2.2.inch - 1.5.inch + 1.inch - 1.8.inch, (-40).degree, 30.degree)

    val kMatchStartToStowed get() = sequential {
        +parallel {
            +ClosedLoopProximalMove((-70).degrees)  
            +ClosedLoopWristMove(40.degrees)
        }
        +ClosedLoopElevatorMove(30.25.inches)
        +kStowed
    }
    val kBackHatchFromLoadingStation get() = SyncedMove.frontToBack
    val kHatchLow get() = everythingMoveTo(19.inches, 0.degrees, 4.degrees)
    val kHatchMid get() = everythingMoveTo(43.inches, 0.degrees, 4.degrees)
    val kHatchHigh get() = everythingMoveTo(65.25.inches, (6).degrees, 6.5.degrees)

    val kCargoIntake get() = everythingMoveTo(25.0.inches, (-44).degrees, (-20).degrees)
    val kCargoShip get() = everythingMoveTo(47.5.inches, (-5).degrees, (-50).degrees)
//    val kCargoShip get() = everythingMoveTo(30.25.inch + 1.2.inch + 24.inch, (-70).degree, 0.degree)

    val kCargoLow get() = everythingMoveTo(20.5.inches, 6.degrees, 16.degrees)
    val kCargoMid get() = everythingMoveTo(45.inches, 6.degrees, 6.degrees)
    val kCargoHigh get() = everythingMoveTo(64.5.inches, 7.degrees, 30.degrees)

    val kStraightDown get() = everythingMoveTo(32.inches, (-70).degrees, (-51).degrees) // sequential {
//        +kHatchMid
//        +ClosedLoopElevatorMove(35.5.inch)
//        +parallel {
//            +ClosedLoopProximalMove((-100).degree)
//            +ClosedLoopWristMove(-50.degree)
//        }
// //        +kStowed
//    }

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

    override fun disableClosedLoopControl() {
        Elevator.disableClosedLoopControl()
        Proximal.disableClosedLoopControl()
        Wrist.disableClosedLoopControl() }

    override fun enableClosedLoopControl() {
        Elevator.enableClosedLoopControl()
        Proximal.enableClosedLoopControl()
        Wrist.enableClosedLoopControl() }

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
        builder.addDoubleProperty("cProxTranslation", { currentState.proximalTranslation().y / kInchToMeter }, { })
        super.initSendable(builder)
    }

    sealed class State {

        object Nothing : State()

        data class Position(
            val elevator: SIUnit<Meter>,
            val proximal: SIUnit<Radian>,
            val wrist: SIUnit<Radian>,
            val isPassedThrough: Boolean = proximal < (-135).degrees,
            val isWristUnDumb: Boolean = false
        ) : State() {

            constructor() : this(20.inches, (-90).degrees, (-45).degrees) // semi-sane numbers?

            fun proximalTranslation() =
                    Translation2d(kProximalLen, proximal.toRotation2d()) + Translation2d(0.meters, elevator)

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
