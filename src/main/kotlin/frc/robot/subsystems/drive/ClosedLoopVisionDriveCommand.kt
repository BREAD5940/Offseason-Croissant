package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import kotlin.math.absoluteValue
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity

class ClosedLoopVisionDriveCommand(private val isFront: Boolean, private val skewCorrect: Boolean = false) : ManualDriveCommand() {

    override fun isFinished() = false

    private var referencePose = Pose2d(0.0.inches, 0.0.inches, 0.degrees.toRotation2d())

    private var prevError = 0.0

    override fun initialize() {
        LimeLight.wantedPipeline = 0
        isActive = true
        referencePose = DriveSubsystem.robotPosition
        LEDs.wantedState = LEDs.State.Off
    }

    private var lastKnownTargetPose: Pose2d? = null

    override fun execute() {

        val newTarget = TargetTracker.getBestTarget(/* referencePose, */ isFront)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

//        var linear = -ManualDriveCommand.speedSource()

//        if (!LimeLight.hasTarget) {
//            super.execute()
        if (lastKnownTargetPose == null) {
//            ElevatorSubsystem.wantedVisionMode = true
            super.execute()
        } else {
            val transform = lastKnownTargetPose.relativeTo(DriveSubsystem.robotPosition)
            var angle = Rotation2d(transform.translation.x, transform.translation.y)
            var linear = -ManualDriveCommand.speedSource()

            if (angle.degrees.absoluteValue > 45) {
                // plz no disable us when going to loading station, kthx
                this.lastKnownTargetPose = null
                super.execute()
            }

//            val angle = LimeLight.lastYaw

            Network.visionDriveAngle.setDouble(angle.degrees)
            Network.visionDriveActive.setBoolean(true)

            // limit linear speed based on elevator height, linear function with height above stowed
            val elevator = Elevator.currentState.position
            if (elevator > 32.inches) {
                // y = mx + b, see https://www.desmos.com/calculator/quelminicu
                // y=\left(\frac{0.35-1}{69-32}\right)\left(x-32\right)+1
                linear *= (-0.0208108 * elevator.inch + 1.66696)
            }

            val multiplier = if (DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter

//            var offset = 0.degree
//            var skew = LimeLight.lastSkew
//            if (skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
//            if (skew > 5.degree && skewCorrect) offset = 0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)

            val error = angle.radians // - offset.radian

            // at 0 speed this should be 1, and at 10ft/sec it should be 2
            // so (0, 1) and (10, 2)
            // y = (2-1)/(10-0) * (x - 0) + 1
            val velocity = (with(DriveSubsystem) {
                leftMotor.encoder.velocity + rightMotor.encoder.velocity
            }).absoluteValue / 2.0
            val scaler = velocity.value * (/* max scaler */ 4.0 - /* min scaler */ 1.0) /
                    (/* velocity at max scaler */10.feet.inMeters()) + 1.0
            var kp = (kCorrectionKp * scaler)
            if (kp > 0.7) kp = 0.7
//            val kp = kCorrectionKp

//            println("kp $kp")

            var turn = kp * error // + kCorrectionKd * (error - prevError)
            if (turn.absoluteValue > maxTurn.value) turn = maxTurn.value

//            val wheelState = curvatureDrive(linear, turn, false)

            DriveSubsystem.setWheelVelocities(DifferentialDriveWheelSpeeds((linear - turn) * multiplier, (linear + turn) * multiplier))

            prevError = error
        }
    }

    override fun end(interrupted: Boolean) {
        Network.visionDriveActive.setBoolean(false)
        isActive = false
        LEDs.setVisionMode(false)
    }

    override fun initSendable(builder: SendableBuilder) {
//        builder.addDoubleProperty("Kp", {kCorrectionKd}, { value -> kCorrectionKp = value})
//        builder.addDoubleProperty("Kd", {kCorrectionKd}, { value -> kCorrectionKd = value})
    }

    companion object {
        var kCorrectionKp = 0.35
        var kCorrectionKd = 0.0
        val maxTurn = 90.degrees.velocity
        var isActive = false
            private set
    }

    init {
        SmartDashboard.putData(this)
    }
}
