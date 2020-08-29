package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.superstructure.Superstructure.getDumbWrist
import kotlin.math.abs
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.min
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.team5940.pantry.lib.SIRotationConstants.kRadianToDegrees
import org.team5940.pantry.lib.WantedState

class SyncedMove(goalAngle: SIUnit<Radian>, proximalMaxVel: SIUnit<AngularVelocity>, wristMaxVel: SIUnit<AngularVelocity>, private val isFrontToBack: Boolean) : FalconCommand(
        Superstructure, Proximal, Wrist
) {

    private val goal = goalAngle + if (isFrontToBack) (-30).degrees else 0.degrees
//    private val goalRotation2d = (goal)
    private val goalWristRotation2d = (goalAngle).radian
    private var lastCommandedProximal: SIUnit<Radian>? = null
    private val proximalVelocity = min(proximalMaxVel.absoluteValue, wristMaxVel.absoluteValue).absoluteValue * 0.8 * (if (isFrontToBack) -1 else 1).toDouble()
    private var lastTime = 0.0
    private var moveIteratorFinished = false
    private var lastObservedState = Superstructure.State.Position()

    constructor(goalAngle: SIUnit<Radian>, isFrontToBack: Boolean) : this(goalAngle, kProximalMaxVel, kWristMaxVel, isFrontToBack)

    override fun initialize() {
        lastCommandedProximal = Superstructure.currentState.proximal
        lastTime = Timer.getFPGATimestamp()
        moveIteratorFinished = false

        println("initial proximal: " + lastCommandedProximal!!)
    }

    override fun execute() {

        println("MOVE I9TERATOR $moveIteratorFinished")

        if (moveIteratorFinished)
            return

        val now = Timer.getFPGATimestamp()
        val dt = (now - lastTime).seconds
        val currentState = Superstructure.currentState
        this.lastObservedState = currentState

        var nextProximal = lastCommandedProximal

        @Suppress("unused")
        operator fun SIUnit<Velocity<Radian>>.times(time: SIUnit<Second>) = SIUnit<Radian>(this.value * time.inSeconds())

        if (abs(Superstructure.getUnDumbWrist(lastObservedState.wrist, lastObservedState.proximal).degree - lastCommandedProximal!!.degree) < 50) {
            val delta = proximalVelocity * dt
            nextProximal = this.lastCommandedProximal!! + (delta)
        }

        if (nextProximal!!.degree < -205) {
            nextProximal = (-205).degrees
            moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 1")
        } else if (nextProximal.degree > 5) {
            nextProximal = (5).degrees
            moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 2")
        }

        println("next proximal: " + nextProximal.degree)

        if (isFrontToBack) {
            if (nextProximal < goal)
                nextProximal = goal
        } else {
            if (nextProximal > goal)
                nextProximal = goal
        }

        if (isFrontToBack) {
            if (nextProximal < goal) {
                this.moveIteratorFinished = true
                println("SETTING MOVE ITERATOR TO TRUE 3")
            }
        } else if (nextProximal > goal) {
            this.moveIteratorFinished = true
            println("SETTING MOVE ITERATOR TO TRUE 4")
        }

        val nextWrist = getDumbWrist(currentState.proximal, currentState.proximal)

        println("next elbow $nextProximal")
        println("next wrist $nextWrist")

        Proximal.wantedState = WantedState.Position(nextProximal)
        Wrist.wantedState = WantedState.Position(nextWrist)

        this.lastCommandedProximal = nextProximal
        lastTime = now

        println("====================")

        println("last observed prox " + lastObservedState.proximal*kRadianToDegrees + " last wrist " +
                lastObservedState.wrist*kRadianToDegrees + " next elbow " + nextProximal +
                " next wrist " + nextWrist)

        println("====================")
    }

    override fun isFinished(): Boolean {
        val toReturn = (Wrist.isWithTolerance(5.0.degrees) &&
                Proximal.isWithTolerance(5.0.degrees)) ||
                moveIteratorFinished

        println("pass thru done? $toReturn")
        println("moveIteratorFinished? $moveIteratorFinished")

        return toReturn
    }

    companion object {
        private val kProximalMaxVel = (190.0 / 360.0 * 2.0 * Math.PI / 0.85).degrees.velocity
        private val kWristMaxVel = kProximalMaxVel // 190d / 360d * 2 * Math.PI;

        val frontToBack
            get() = sequential {
                +PrintCommand("passing thru front to back")
                +InstantCommand(Runnable { IntakeSubsystem.wantsOpen = false }, IntakeSubsystem)
                +ClosedLoopElevatorMove(36.inches)
                +SyncedMove((-160).degrees, true)
                +parallel {
                    +ClosedLoopProximalMove((-193.0).degrees)
                    +ClosedLoopWristMove((-112.0).degrees)
                }
                +ClosedLoopElevatorMove(19.5.inches)
            }

        val backToFront
            get() = sequential {
                +PrintCommand("passiing thru back to front")
                +InstantCommand(Runnable { IntakeSubsystem.wantsOpen = false }, IntakeSubsystem)
                +ClosedLoopElevatorMove(36.inches)
                +SyncedMove(0.0.degrees, false)
                +parallel {
                    +ClosedLoopProximalMove(0.0.degrees)
                    +ClosedLoopWristMove(0.0.degrees)
                    +ClosedLoopElevatorMove(26.0.inches)
                }
            }

        val shortPassthrough
            get() = sequential {
                +ClosedLoopElevatorMove(36.inches)
                +SyncedMove(0.0.degrees, false) }
    }
}
