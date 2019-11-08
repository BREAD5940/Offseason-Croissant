package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.Source
import org.team5940.pantry.lib.WantedState

class ClosedLoopElevatorMove(private val target: Source<Length>) : FalconCommand(Elevator) {

    constructor(target: Length) : this({ target })

    override fun initialize() {
        Elevator.wantedState = WantedState.Position(target())
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Elevator.wantedState = WantedState.Position(Elevator.currentState.position)
        }
    }

    override fun isFinished() = Elevator.isWithTolerance(1.inch)

    // : Boolean {
//        val error = abs(target - Elevator.currentState.position) / SILengthConstants.kInchToMeter
//        return error < 1.0
//    }
}

class ClosedLoopProximalMove(private val target: SIUnit<Radian>) : FalconCommand(Proximal) {

    override fun initialize() {
        Proximal.wantedState = WantedState.Position(target)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Proximal.wantedState = WantedState.Position(Proximal.currentState.position)
        }
    }

    override fun isFinished() = Proximal.isWithTolerance(3.0.degree) // toDegrees(abs(target - Proximal.currentState.position)) < 5.0
}

class ClosedLoopSourceProximalMove(private val endState: SIUnit<Radian>, private val target: Source<SIUnit<Radian>>) : FalconCommand(Proximal) {

    override fun execute() {
        Proximal.wantedState = WantedState.Position(target())
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Proximal.wantedState = WantedState.Position(Proximal.currentState.position)
        }
    }

    override fun isFinished() = (endState - Proximal.currentState.position).absoluteValue < 3.degree
    // Proximal.isWithTolerance(3.0.degree) // toDegrees(abs(target - Proximal.currentState.position)) < 5.0
}

class ClosedLoopWristMove(private val target: SIUnit<Radian>) : FalconCommand(Wrist) {

    override fun initialize() {
        Wrist.wantedState = WantedState.Position(target)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // stop moving
            Wrist.wantedState = WantedState.Position(Wrist.currentState.position)
        }
    }

    override fun isFinished() = Wrist.isWithToleranceWithOffset(5.0.degree) // toDegrees(abs(target - Wrist.currentState.position)) < 5.0
}

// class JogElevator : FalconCommand(Superstructure, Elevator) {
//
//    companion object {
//        val upSource by lazy { (Controls.operatorFalconHID.getRawButton(9)) }
//        val downSource by lazy { (Controls.operatorFalconHID.getRawButton(11)) }
//    }
//
//    private var initPosition: Double? = null
//
//    override fun initialize() {
//        initPosition = Elevator.currentState.position
//    }
//
//    override fun execute() {
//        val upPower = if (upSource()) 1 else -1
//        val downPower = if (downSource()) -1 else 1
//        val totalPower = (upPower + downPower) * 0.3 * SILengthConstants.kInchToMeter
//
//        if (totalPower.absoluteValue < 0.05) return
//
//        Elevator.wantedState = WantedState.Position(
//                let {
//                    initPosition?.plus(totalPower) ?: Elevator.currentState.position // yeet it's an Elvis
//                }
//        )
//    }
// }
