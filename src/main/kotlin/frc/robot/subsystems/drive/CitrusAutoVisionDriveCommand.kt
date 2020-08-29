package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import frc.robot.Constants
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.vision.TargetTracker
import kotlin.math.absoluteValue
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters

class CitrusAutoVisionDriveCommand(private val isStowed: Boolean, private val skewCorrect: Boolean = true) : FalconCommand(DriveSubsystem) {

    private var noTarget = 0
    private var cantFindTarget = false
    private var inRange = false
    private var inRangeTime = -1.0
    private val kEndTimeout = 0.6
    private var prevAngleError = 0.degrees.toRotation2d()
    private var lastKnownTargetPose: Pose2d? = null

    /**
     * Target distance from the center of the robot to the vision taret
     */
    private val targetDistance = (if (isStowed) Constants.kCenterToForwardIntakeStowed else Constants.kCenterToForwardIntake)
            .translation.x.absoluteValue.meters

    override fun end(interrupted: Boolean) {
        noTarget = 0
        cantFindTarget = false
        inRange = false
        inRangeTime = -1.0
//        prevAngleError = 0.degree.toRotation2d()
    }

    override fun initialize() {
        noTarget = 0
        cantFindTarget = false
        inRange = false
//        prevAngleError = 0.degree.toRotation2d()
        lastKnownTargetPose = null
    }

    override fun execute() {

        val newTarget = TargetTracker.getBestTarget(false)
        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) this.lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        if (lastKnownTargetPose == null) {
            noTarget++
            if (noTarget > 40) cantFindTarget = true
            return
        }

        // we know we have a new target
        noTarget = 0

        val offset = if (!skewCorrect) 0.degrees else {
            var skew = LimeLight.lastSkew
            if (skew > (-45).degrees) skew = skew.absoluteValue else skew += 90.degrees
            if (skew > 5.degrees) {
                0.05.degrees * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)
            } else 0.degrees
        }

        val transform = lastKnownTargetPose.relativeTo(DriveSubsystem.robotPosition)
        val angle = Rotation2d(transform.translation.x, transform.translation.y)

        // idk man maybe 1 feet per second at 1 ft of error?
        val currentDistance = LimeLight.estimateDistance()
        val linear = (currentDistance - Constants.kCenterToFrontCamera.translation.x.meters - targetDistance).velocity * kLinearKp // TODO tune

        // P loop on heading
        val turn = kCorrectionKp * angle.radians + kCorrectionKd * (angle - prevAngleError).radians

        DriveSubsystem.setWheelVelocities(DifferentialDriveWheelSpeeds(linear.value + turn, linear.value - turn))

        inRange = currentDistance < targetDistance + 2.inches
        if (inRange && inRangeTime > 0.0) inRangeTime = Timer.getFPGATimestamp()

        prevAngleError = angle
    }

    override fun isFinished() = cantFindTarget || (inRange && (inRangeTime - Timer.getFPGATimestamp()) > kEndTimeout)

    //    override fun initSendable(builder: SendableBuilder) {
//        builder.addDoubleProperty("kp", { kCorrectionKp }, {_new -> kCorrectionKp = _new})
//        builder.addDoubleProperty("kd", { kCorrectionKd }, {_new -> kCorrectionKd = _new})
//        super.initSendable(builder)
//    }

    companion object {
        var kCorrectionKp = 1.9
        var kCorrectionKd = 14.0
        var kLinearKp = 1.0
    }
}
