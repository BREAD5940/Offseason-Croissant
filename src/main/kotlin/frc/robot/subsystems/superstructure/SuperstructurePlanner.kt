package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import frc.robot.auto.routines.withExit
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.min
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import java.awt.Color
import java.lang.IllegalStateException
import kotlin.math.min

object SuperstructurePlanner {

    private val kCrossbar = 33.inch
    private val kMinimumSafeSyncedProximal = (-75).degree
    private val kOutsideFrame = (-35).degree
//    val kProximalLen = 32.inch.meter

    private fun planPath(currentState: SuperstructureState, goalState: SuperstructureState) = sequential {

        // first check passthrough -- remember, if we do pass through, our "current state" will change!
        val needsPassthrough = !currentState.isPassedThrough and goalState.isPassedThrough

        if (needsPassthrough) {
            val worstCaseProximalAfterPassthrough =
                    worstCaseProximalTipElevation(SuperstructureState(kCrossbar, kMinimumSafeSyncedProximal, 0.degree), goalState)

            if (worstCaseProximalAfterPassthrough > 2.inch && goalState.proximal > kMinimumSafeSyncedProximal) {
                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar
                println("                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar")
                +ClosedLoopElevatorMove(kCrossbar)
                +parallel {
                    +SyncedMove(0.0.degree, false)
                    +sequential {
                        +WaitUntilCommand { Superstructure.currentState.proximal > (-75).degree }
                        +ClosedLoopElevatorMove(goalState.elevator)
                    }
                } // these have to be outside the parallel group because SyncedMove reserves the proximal and wrist
                +ClosedLoopWristMove(goalState.wrist)
                +ClosedLoopProximalMove(goalState.proximal)
            } else if (worstCaseProximalAfterPassthrough < 2.inch &&
                    goalState.proximal > kOutsideFrame) {
                // arm needs to move all the way out before we start moving the elevator
                println("                // arm needs to move all the way out before we start moving the elevator")
                +ClosedLoopElevatorMove(kCrossbar)
                +SyncedMove(0.0.degree, false)
                +parallel {
                    +ClosedLoopElevatorMove(goalState.elevator)
                    +ClosedLoopProximalMove(goalState.proximal)
                    +ClosedLoopWristMove(goalState.wrist)
                }
            } else if (goalState.elevator < kCrossbar &&
                    goalState.proximal < kMinimumSafeSyncedProximal &&
                    goalState.proximalTranslation().y > 2.inch) {
                // below the crossbar and below -75 degrees
                println("// below the crossbar and below -75 degrees")
                +ClosedLoopElevatorMove(kCrossbar)
                +SyncedMove(0.0.degree, false)
                +parallel {
                    +ClosedLoopProximalMove(goalState.proximal)
                    +ClosedLoopWristMove(goalState.wrist)
                }
                +ClosedLoopElevatorMove(goalState.elevator)
            } else {
                throw IllegalStateException("Move is impossible, maybe!")
            }
        } else {
            // doesn't need passthrough, go through the checks again
            val worstCaseProximal =
                    worstCaseProximalTipElevation(currentState, goalState)

            val minProxAngle = min(currentState.proximal, goalState.proximal)

            if ((worstCaseProximal > -3.inch ||
                            minProxAngle > kOutsideFrame) && /* exception for cargo grab */
                    goalState.proximal > kMinimumSafeSyncedProximal) {
                // synced safe, as it's an upward(ish, technically) move that's outside the crossbar
                println("// synced safe, as it's an upward(ish, technically) move that's outside the crossbar")
                +parallel {
                    +ClosedLoopElevatorMove(goalState.elevator)
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
            }

            // next arm first vs elevator first
            else if (
                    minProxAngle < kMinimumSafeSyncedProximal &&
                    goalState.proximal > kOutsideFrame) {
                // arm first if it's a downward move to below electronics or if we're inside the crossbar
                println("// arm first if it's a downward move to below electronics or if we're inside the crossbar")
                +sequential {
                    val armMove = parallel { +ClosedLoopWristMove(goalState.wrist)
                        +ClosedLoopProximalMove(goalState.proximal) }
                    +armMove
//                    +sequential {
//                        +WaitUntilCommand { Superstructure.currentState.proximal > kOutsideFrame }.withExit { armMove.isFinished }
                    +ClosedLoopElevatorMove(goalState.elevator)
//                    }
                }
            } else if (currentState.elevator > goalState.elevator &&
                    goalState.elevator < kCrossbar &&
                    currentState.proximal > kMinimumSafeSyncedProximal &&
                    goalState.proximal < kMinimumSafeSyncedProximal
                    ) {
                // weird handler for downward moves to somewhere within the crossbar
                println("// weird handler for downward moves to somewhere within the crossbar")

                // move the elevator down then move joints
                +parallel {
                    val elevatorMove = ClosedLoopElevatorMove(goalState.elevator)
                    +elevatorMove
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(min(goalState.proximal, kMinimumSafeSyncedProximal))
                    +sequential {
                        +WaitUntilCommand { Superstructure.currentState.elevator < kCrossbar }.withExit { elevatorMove.isFinished }
                        +ClosedLoopWristMove(goalState.wrist)
                        +ClosedLoopProximalMove(goalState.proximal)
                    }
                }
            } else if (worstCaseProximal < -3.inch &&
                    currentState.proximal > kOutsideFrame // &&
                    /*goalState.proximal < kOutsideFrame*/) {
                // elevator first (i.e. coming from cargo grab to stowed)
                +ClosedLoopElevatorMove(goalState.elevator)
                +parallel {
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
            } else {
                throw IllegalStateException("Cannot find a safe path from ${currentState.asString()} to ${goalState.asString()}")
            }
        }
    }

    /**
     * Code from 2019 inseason
     */
    private fun planOldPath(currentState: SuperstructureState, goalState: SuperstructureState) = sequential {

        // check passthrough
        val needsPassthrough = currentState.proximal < (-140).degree
        if (needsPassthrough) {
            +SyncedMove.shortPassthrough
        }

        // choose between everything at the same time, elevator first or arm first
        val proximalThreshold = (-73).degree
        val nowOutsideCrossbar = currentState.proximal > proximalThreshold
        val willBeOutsideCrossbar = goalState.proximal > proximalThreshold
        var mightHitElectronics = (goalState.elevator < 23.inch && goalState.proximal > proximalThreshold) || (goalState.elevator < 31.inch && goalState.proximal < proximalThreshold) // TODO check angles?

        val proximalStartSafe = currentState.proximal > -80.degree
        val proximalEndSafe = goalState.proximal > -80.degree
        val startHighEnough = currentState.elevator > 18.inch
        val endHighEnough = goalState.elevator > 31.inch

        val needsExceptionForCargoGrab = currentState.proximal > (-62).degree && currentState.elevator > 36.inch && goalState.proximal > (-62).degree

        val safeToMoveSynced = (nowOutsideCrossbar && willBeOutsideCrossbar && (!mightHitElectronics || needsExceptionForCargoGrab)) ||
                (proximalStartSafe && proximalEndSafe && startHighEnough && endHighEnough)

        val needsLongMoveAboveCrossbarException = currentState.elevator > 54.inch
                && goalState.elevator < 54.inch && goalState.proximal < (-60).degree

        if (safeToMoveSynced && !needsLongMoveAboveCrossbarException) {
            // yeet everything at the same time
            +parallel {
                +ClosedLoopElevatorMove(goalState.elevator)
                +ClosedLoopWristMove(goalState.wrist)
                +ClosedLoopProximalMove(goalState.proximal)
            }
        } else if(safeToMoveSynced && needsLongMoveAboveCrossbarException) {
            +parallel {
                +ClosedLoopElevatorMove(goalState.elevator)
                +ClosedLoopWristMove(goalState.wrist)
                +ClosedLoopSourceProximalMove(goalState.proximal) {
                    @Suppress("RemoveRedundantQualifierName")
                    if(Superstructure.currentState.elevator < 54.inch) { goalState.proximal } else { (-60).degree }
                }
            }
        }
        else {
            // choose between arm first or elevator first
            val proximalThresh = (-18).degree
            val startAboveSafe = goalState.elevator > 36.inch
//            val endAboveSafe = currentState.elevator > 36.inch
            val nowOutsideFrame = currentState.proximal > proximalThresh
            val willBeOutsideFrame = goalState.proximal > proximalThresh

            val shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) || (nowOutsideFrame && willBeOutsideFrame) ||
                    (((-35).degree >= currentState.proximal && currentState.proximal >= (-90).degree) &&
                        ((-50).degree >= goalState.proximal && goalState.proximal >= (-100).degree))
            if (shouldMoveElevatorFirst) {
                +ClosedLoopElevatorMove(goalState.elevator)
                +parallel {
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
            } else {
                +parallel {
                    +ClosedLoopWristMove(goalState.wrist)
                    +ClosedLoopProximalMove(goalState.proximal)
                }
                +ClosedLoopElevatorMove(goalState.elevator)
            }
        }
        +InstantCommand(Runnable {
            LEDs.wantedState = LEDs.State.Blink(0.175.second, Color.GREEN)
            GlobalScope.launch {
                delay(500)
                LEDs.wantedState = LEDs.State.Off
                delay(250)
                LEDs.wantedState = LEDs.State.Default
            }
        })
    }

    fun worstCaseProximalTipElevation(currentState: SuperstructureState, goalState: SuperstructureState): Length {
        val worstArmTranslation = Translation2d(kProximalLen, min(currentState.proximal, goalState.proximal).minus(5.degree).toRotation2d())
        return min(currentState.elevator, goalState.elevator) + worstArmTranslation.y
    }

    fun everythingMoveTo(
        elevator: Length,
        proximal: SIUnit<Radian>,
        wrist: SIUnit<Radian>
    ) = everythingMoveTo(
            SuperstructureState(elevator, proximal, wrist))

    fun everythingMoveTo(goalState: SuperstructureState): CommandBase =
            object : FalconCommand(Superstructure, Proximal, Wrist, Elevator) {
                // This whole {} thing is a Supplier<Command> that will return a Command that moves everything safely (hopefully)

                override fun getName() = "move to ${goalState.asString()}"

                var path: CommandBase? = null
                var pathStarted = false

                override fun initialize() {
                    path = planOldPath(currentState = Superstructure.currentState, goalState = goalState)
                    path!!.initialize()
                    pathStarted = true
                }

                override fun execute() {
                    path!!.execute()
                }

                override fun end(interrupted: Boolean) {
                    path?.end(interrupted)
                    path = null
                    pathStarted = false
                }

                override fun isFinished(): Boolean {
                    val path = this.path
                    return path?.isFinished ?: pathStarted // if the path is null, check that it's started, otherwise call the path's isFinished() method
                }
            }
}
