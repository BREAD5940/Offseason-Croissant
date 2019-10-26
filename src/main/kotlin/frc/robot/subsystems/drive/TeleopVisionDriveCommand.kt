// implementation from Team 5190 Green Hope Robotics

package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.absoluteValue

class TeleopVisionDriveCommand(private val isFront: Boolean, private val skewCorrect: Boolean = true) : ManualDriveCommand() {

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
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition
            var angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)
            val distance = transform.translation.norm.feet.absoluteValue

            // limit linear speed based on elevator height, linear function with height above stowed
            val elevator = Elevator.currentState.position
            if (elevator > 32.inch && elevator < 55.inch) {
                // y = mx + b, see https://www.desmos.com/calculator/quelminicu
                source *= (-0.0216 * elevator.inch + 1.643)
            }

            if (distance < 6) {
                source *= (distance + 1) / 6.0
            }

            val offset = if(!skewCorrect) 0.degree else {
                var skew = LimeLight.lastSkew
                if(skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
                if(skew > 5.degree) {
                    0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)
                } else 0.degree
            }
            angle -= offset.toRotation2d()

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            val angleError = angle + if (isFront) 0.degree.toRotation2d() else Math.PI.radian.toRotation2d() - 1.7.degree.toRotation2d()

//            if (angleError.degree.absoluteValue > 45) {
//                // plz no disable us when going to loading station, kthx
//                this.lastKnownTargetPose = null
//            }

            val error = angleError.radian

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            DriveSubsystem.tankDrive(source - turn, source + turn)

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