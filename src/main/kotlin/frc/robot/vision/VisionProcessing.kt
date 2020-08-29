package frc.robot.vision

import com.google.gson.JsonObject
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.Constants.kCenterToBackCamera
import frc.robot.Constants.kCenterToFrontCamera
import frc.robot.Constants.kRobotLength
import frc.robot.Constants.kRobotWidth
import frc.robot.subsystems.drive.DriveSubsystem
import kotlin.math.absoluteValue
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d

object VisionProcessing {

    fun processData(visionData: VisionData) {

        // TODO implement this
//        if (visionData.isFront && ElevatorSubsystem.position in Constants.kElevatorBlockingCameraRange) {
//            // Vision cannot see through the carriage
//            return
//        }

        val robotPose = DriveSubsystem.robotPosition // [visionData.timestamp.second]

        val samples = visionData.targets
                .asSequence()
                .mapNotNull { // make sure the return of processReflectiveTape is non-null
                    processReflectiveTape(
                            it,
                            if (visionData.isFront) kCenterToFrontCamera else kCenterToBackCamera
                    )
                }
                .filter { // make sure that the vision target Pose2d is outside the robot
                    // We cannot be the vision target :)
                    it.translation.x.absoluteValue > (kRobotLength / 2.0 - 5.inches).inMeters() ||
                            it.translation.y.absoluteValue > (kRobotWidth / 2.0).inMeters()
                }
                .map { robotPose + it } // sum the robotpose and the targetpose to get a global pose
                .toList() // make it into a List

        TargetTracker.addSamples(
            visionData.timestamp, samples
        )
    }

    private fun processReflectiveTape(data: JsonObject, transform: Pose2d): Pose2d? {
        val angle = data["angle"].asDouble.degrees
        val rotation = -data["rotation"].asDouble.degrees + angle + 180.degrees
        val distance = data["distance"].asDouble.inches

//        println("${distance.inch}, ${angle.degree}")

        return transform + Pose2d(Translation2d(distance, angle.toRotation2d()), rotation.toRotation2d())
    }

//    val kCenterToFrontCamera = Pose2d((-1.75).inch, 0.0.inch, 0.degree)
//    val kCenterToBackCamera = Pose2d((6.25).inch, 0.0.inch, 180.degree) // make sure these numbers are right
//    val kRobotWidth = 30.inch
//    val kRobotLength = 30.inch
}

fun Pose2d.transformBy(other: Pose2d) = this.transformBy(edu.wpi.first.wpilibj.geometry.Transform2d(
        other.translation, other.rotation
))

operator fun Pose2d.plus(other: Pose2d): Pose2d = this.transformBy(other)!!
