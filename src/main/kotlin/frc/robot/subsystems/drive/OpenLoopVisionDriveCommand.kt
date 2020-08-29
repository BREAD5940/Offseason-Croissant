// implementation from Team 5190 Green Hope Robotics

package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import kotlin.math.absoluteValue
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports

class OpenLoopVisionDriveCommand(private val isFront: Boolean, private val skewCorrect: Boolean = true) : ManualDriveCommand() {

    override fun isFinished() = false

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
//        LEDs.setVisionMode(/*true*/)
        LEDs.wantedState = LEDs.State.Off
//        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0)
//        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0)
    }

    override fun execute() {

        val newTarget = TargetTracker.getBestTarget(isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        var source = -speedSource()

        if (lastKnownTargetPose == null) {
//            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
//            ElevatorSubsystem.wantedVisionMode = false
            val transform = lastKnownTargetPose.relativeTo(DriveSubsystem.robotPosition)
            var angle = Rotation2d(transform.translation.x, transform.translation.y)
            val distance = transform.translation.norm.absoluteValue / kFeetToMeter

            // limit linear speed based on elevator height, linear function with height above stowed
            val elevator = Elevator.currentState.position
            if (elevator > 32.inches && elevator < 55.inches) {
                // y = mx + b, see https://www.desmos.com/calculator/quelminicu
                source *= (-0.0216 * elevator.inch + 1.643)
            }

            if (distance < 6) {
                source *= (distance + 1.0) / 6.0
            }

            val offset = if (!skewCorrect) 0.degrees else {
                var skew = LimeLight.lastSkew
                if (skew > (-45).degrees) skew = skew.absoluteValue else skew += 90.degrees
                if (skew > 5.degrees) {
                    0.05.degrees * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)
                } else 0.degrees
            }
            angle -= offset.toRotation2d()

            Network.visionDriveAngle.setDouble(angle.degrees)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (isFront) 0.degrees.toRotation2d() else Math.PI.radians.toRotation2d() - 1.7.degrees.toRotation2d()

//            if (angleError.degree.absoluteValue > 45) {
//                // plz no disable us when going to loading station, kthx
//                this.lastKnownTargetPose = null
//            }

            val error = angleError.radians

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            DriveSubsystem.setPercent(source - turn, source + turn)

            prevError = error
        }
    }

    override fun end(interrupted: Boolean) {
        Network.visionDriveActive.setBoolean(false)
        this.lastKnownTargetPose = null
//        ElevatorSubsystem.wantedVisionMode = false
        isActive = false
        LEDs.setVisionMode(false)
//        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1)
//        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1)
    }

    override fun initSendable(builder: SendableBuilder) {

//        builder.addDoubleProperty("forwardKp", { kCorrectionKp }, { newP -> kCorrectionKp = newP })
//        builder.addDoubleProperty("forwardKd", { kCorrectionKd }, { newD -> kCorrectionKd = newD })

        super.initSendable(builder)
    }

    companion object {
        var kCorrectionKp = 0.8 * 1.2 * 1.5
        var kCorrectionKd = 8.0
        var isActive = false
            private set
    }
}
