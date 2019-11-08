package frc.robot.subsystems.superstructure

import frc.robot.Constants
import frc.robot.Constants.SuperStructureConstants.kProximalLen
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.team5940.pantry.lib.WantedState

/**
 * Thrust the wrist out. This reduces the proximal cruise velocity and uses position closed loop on the
 * Elevator and Wrist such that the proximal remains at a fixed [mWristHeightToMaintain] out to a given
 * [proximalAngle].
 * @param mWristHeightToMaintain the wrist height to maintain
 * @param proximalAngle the proximal angle to reach
 */
class PrismaticWristPokeMove(private val mWristHeightToMaintain: SIUnit<Meter>, private val proximalAngle: SIUnit<Radian>) : FalconCommand(Superstructure, Elevator, Proximal, Wrist) {

    override fun initialize() {
        Elevator.setPositionMode()
        Wrist.setPositionMode()
        Proximal.configureThrust(Constants.SuperStructureConstants.kProximalThrustVelocity)
    }

    override fun execute() {
        val elevatorHeightDiff = kProximalLen * Proximal.activeTrajectoryPosition.toRotation2d().sin
        val elevatorHeight = (mWristHeightToMaintain - elevatorHeightDiff)
                .coerceIn(Constants.SuperStructureConstants.kElevatorRange)
        val wristPos = Superstructure.getDumbWrist(0.degree, Superstructure.currentState.proximal)

        Elevator.wantedState = WantedState.Position(elevatorHeight)
        Proximal.wantedState = WantedState.Position(proximalAngle)
        Wrist.wantedState = WantedState.Position(wristPos)
    }

    override fun isFinished() = Proximal.isWithTolerance(proximalAngle, 2.degree)

    override fun end(interrupted: Boolean) {
        Proximal.motor.setClosedLoopGains()
        Elevator.setMotionMagicMode()
        Wrist.setMotionMagicMode()
    }
}
