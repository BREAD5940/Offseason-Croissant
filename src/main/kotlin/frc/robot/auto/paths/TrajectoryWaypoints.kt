package frc.robot.auto.paths

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.robot.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degrees

object TrajectoryWaypoints {

    /** Measured Field Coordinates **/

    // Habitat Zone
    val kHabitatL2RX = 48.00.inches
    val kHabitatL2BY = 97.00.inches
    val kHabitatL1RX = 95.28.feet
    val kHabitatL1Platform = Rectangle2d(Translation2d(4.feet, 7.feet), Translation2d(8.feet, 20.feet))
    val kRampHypotenuse = .4.inches

    // Cargo Ship
    val kCargoShipFL = Pose2d(220.25.inches, 172.88.inches, 0.degrees)
    val kCargoShipFR = Pose2d(220.25.inches, 151.12.inches, 0.degrees)
    val kCargoShipS1 = Pose2d(260.75.inches, 133.13.inches, 90.degrees)
    val kCargoShipS2 = Pose2d(282.55.inches, 133.13.inches, 90.degrees)
    val kCargoShipS3 = Pose2d(304.30.inches, 133.13.inches, 90.degrees)

    // Rocket
    val kRocketN = Pose2d(214.57.inches, 19.57.inches, (-28.75).degrees)
    val kRocketF = Pose2d(244.00.inches, 19.57.inches, (-151.25).degrees)
    val kRocketBay = Pose2d(229.28.inches, 27.50.inches, (-90).degrees)

    // Loading Station
    val kLoadingStation = Pose2d(0.inches, 25.72.inches, 0.degrees)
    val kLoadingStationReversed = Pose2d(0.inches, 25.72.inches, 180.degrees)

    // Depot
    val kDepotBRCorner = Pose2d(47.inches, 78.396.inches, (-25).degrees)

    /** Robot Starting Locations **/

    // Determine the starting X value for the robot.
    private val kStartX = kHabitatL2RX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 - kRampHypotenuse

    // Starting on Level 1 HAB on the right side.
    val kSideStart = Pose2d(
            kHabitatL2RX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 - kRampHypotenuse,
            kHabitatL2BY + Constants.kBumperThickness + Constants.kRobotWidth / 2.0,
            0.degrees
    )

    val kSideStartReversed = Pose2d(kSideStart.translation, 180.degrees)

    // Starting on Level 1 HAB in the center.
    val kCenterStart = Pose2d(kStartX, 13.5.feet)

    data class Waypoint(
        val trueLocation: Pose2d,
        val transform: Pose2d = Pose2d(),
        val translationalOffset: Translation2d = Translation2d(),
        val rotationalOffset: Rotation2d = Rotation2d(0.0)
    ) {

        val trueAndTransform = trueLocation + transform

        fun transformBy(other: Pose2d) = Waypoint(trueLocation, transform.transformBy(other), translationalOffset, rotationalOffset)

        val position = Pose2d(
                trueAndTransform.translation + translationalOffset,
                trueAndTransform.rotation + rotationalOffset
        )
    }
}
