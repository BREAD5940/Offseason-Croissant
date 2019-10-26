package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.subsystems.sensors.LimeLight
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inch

class CitrusAutoVisionDriveCommand(private val isStowed: Boolean, private val skewCorrect: Boolean = true): FalconCommand(DriveSubsystem) {

    private var noTarget = 0
    private var cantFindTarget = false
    private var inRange = false
    private var inRangeTime = -1.0
    private val kEndTimeout = 0.6
    private var prevAngleError = 0.degree.toRotation2d()
    /**
     * Target distance from the center of the robot to the vision taret
     */
    private val targetDistance = (if(isStowed) Constants.kCenterToForwardIntakeStowed else Constants.kCenterToForwardIntake).translation.x.absoluteValue

    override fun end(interrupted: Boolean) {
        noTarget = 0
        cantFindTarget = false
        inRange = false
        inRangeTime = -1.0
        prevAngleError = 0.degree.toRotation2d()
    }

    override fun initialize() {
        noTarget = 0
        cantFindTarget = false
        inRange = false
        prevAngleError = 0.degree.toRotation2d()
    }

    override fun execute() {
        if(!LimeLight.hasTarget) {
            noTarget++
            if(noTarget > 40) cantFindTarget = true
            return
        }
        // we know we have a new target
        noTarget = 0

        val offset = if(!skewCorrect) 0.degree else {
            var skew = LimeLight.lastSkew
            if(skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
            if(skew > 5.degree) {
                0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)
            } else 0.degree
        }

        val globalPose = (LimeLight.lastYaw - offset).toRotation2d() + DriveSubsystem.localization[LimeLight.currentState.timestamp].rotation
        val angleError = (globalPose - DriveSubsystem.robotPosition.rotation)

        // idk man maybe 1 feet per second at 1 ft of error?
        val currentDistance = LimeLight.estimateDistance()
        val linear = (currentDistance - Constants.kCenterToFrontCamera.translation.x - targetDistance).velocity * kLinearKp // TODO tune

        // P loop on heading
        val turn = kCorrectionKp * angleError.radian + kCorrectionKd * (angleError - prevAngleError).radian

        DriveSubsystem.setWheelVelocities(DifferentialDrive.WheelState(linear.value + turn, linear.value - turn))

        inRange = currentDistance < targetDistance + 2.inch
        if(inRange && inRangeTime > 0.0) inRangeTime = Timer.getFPGATimestamp()

        prevAngleError = angleError
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