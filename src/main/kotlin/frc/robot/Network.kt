/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package frc.robot

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import frc.robot.auto.Autonomous
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.SuperstructureState
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser
import org.team5940.pantry.lib.Updatable

object Network : Updatable {

    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("CROISSANT")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0)

//    private val localizationLayout = mainShuffleboardDisplay.getLayout("Localization", BuiltInLayouts.kList)
//            .withSize(2, 2)
//            .withPosition(2, 0)

    private val visionLayout = mainShuffleboardDisplay.getLayout("Vision", BuiltInLayouts.kGrid)
            .withSize(3, 3)
            .withPosition(0, 2)

//    private val driveSubsystemLayout = mainShuffleboardDisplay.getLayout("Drive", BuiltInLayouts.kGrid)
//            .withSize(2, 2)
//            .withPosition(4, 0)

    private val elevatorSubsystemLayout = mainShuffleboardDisplay.getLayout("Elevator", BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(6, 0)

    private val jointPosition = elevatorSubsystemLayout.add("total state", SuperstructureState().asString()).entry
    private val elevatorPosition = elevatorSubsystemLayout.add("Position (in)", 0.0).entry
    private val elevatorSetpoint = elevatorSubsystemLayout.add("Setpoint (in)", 0.0).entry
    private val elevatorVelocity = elevatorSubsystemLayout.add("Velocity (ips)", 0.0).entry

    val autoVisionP = visionLayout.add("Auto kP", 0.0).entry
    val autoVisionD = visionLayout.add("Auto kD", 0.0).entry

    val visionDriveAngle = visionLayout.add("Vision Drive Angle", 0.0).entry
    val visionDriveActive = visionLayout.add("Vision Drive Active", false).entry

    init {

        startingPositionChooser.setDefaultOption(Autonomous.StartingPositions.CENTER.name, Autonomous.StartingPositions.CENTER)
        autoModeChooser.setDefaultOption(Autonomous.Mode.DO_NOTHING.name, Autonomous.Mode.DO_NOTHING)

        // Put choosers on dashboard
        autoLayout.add(
                "Auto Mode",
                autoModeChooser
        )
        autoLayout.add(
                "Starting Position",
                startingPositionChooser
        )
    }

    override fun update() {
//        elevatorPosition.setDouble(Elevator.currentState.position.inch)
//        elevatorVelocity.setDouble(Elevator.currentState.velocity.inchesPerSecond)
//        elevatorSetpoint.setDouble(let {
//            val wantedState = Elevator.wantedState as? WantedState.Position<*> ?: return@let 0.0
//            wantedState.targetPosition.value / kInchToMeter
//        })
        jointPosition.setString(Superstructure.currentState.toString())
    }
}
