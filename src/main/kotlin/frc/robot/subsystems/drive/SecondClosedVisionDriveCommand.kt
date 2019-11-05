package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Network
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.LEDs
import frc.robot.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.absoluteValue

class SecondClosedVisionDriveCommand(private val isFront: Boolean, private val skewCorrect: Boolean = false) : ManualDriveCommand() {

    override fun isFinished() = false

    private var referencePose = Pose2d()

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
            val transform = lastKnownTargetPose inFrameOfReferenceOf DriveSubsystem.robotPosition
            var angle = Rotation2d(transform.translation.x.meter, transform.translation.y.meter, true)
            var linear = -ManualDriveCommand.speedSource()

            if (angle.degree.absoluteValue > 45) {
                // plz no disable us when going to loading station, kthx
                this.lastKnownTargetPose = null
                super.execute()
            }

//            val angle = LimeLight.lastYaw

            Network.visionDriveAngle.setDouble(angle.degree)
            Network.visionDriveActive.setBoolean(true)

            // limit linear speed based on elevator height, linear function with height above stowed
            val elevator = Elevator.currentState.position
            if (elevator > 32.inch) {
                // y = mx + b, see https://www.desmos.com/calculator/quelminicu
                // y=\left(\frac{0.35-1}{69-32}\right)\left(x-32\right)+1
                linear *= (-0.0208108 * elevator.inch + 1.66696)
            }

            val multiplier = if (DriveSubsystem.lowGear) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter

//            var offset = 0.degree
//            var skew = LimeLight.lastSkew
//            if (skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
//            if (skew > 5.degree && skewCorrect) offset = 0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13)

            val error = angle.radian * -1.0 //- offset.radian

            // at 0 speed this should be 1, and at 10ft/sec it should be 2
            // so (0, 1) and (10, 2)
            // y = (2-1)/(10-0) * (x - 0) + 1
//            val velocity = (with(DriveSubsystem) {
//                leftMotor.encoder.velocity + rightMotor.encoder.velocity
//            }).absoluteValue / 2.0
//            val scaler = velocity.value * (/* max scaler */ 4.0 - /* min scaler */ 1.0) /
//                    (/* velocity at max scaler */10.feet.meter) + 1.0
//            var kp = (kCorrectionKp * scaler)
//            if (kp > 0.7) kp = 0.7
//            val kp = kCorrectionKp

//            println("kp $kp")

            val isQuickTurn = linear.absoluteValue < 0.1
            val turn = if(isQuickTurn) {
                kStaticKp * error + kStaticKd * (error - prevError)
            } else {
                kDynamicKp * error + kDynamicKd * (error - prevError)
            }

            println("QUICK TURN? $isQuickTurn turn $turn")

            var wheelSpeeds = curvatureDrive(linear, turn, isQuickTurn)
            wheelSpeeds = DifferentialDrive.WheelState(wheelSpeeds.left * multiplier, wheelSpeeds.right * multiplier)

            DriveSubsystem.setWheelVelocities(wheelSpeeds)

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

        val kStaticKp = 1.0 * 0.85
        val kStaticKd = 4.0

        val kDynamicKp = 4.5
        val kDynamicKd = 10.0


        var isActive = false
            private set
    }

    init {
        SmartDashboard.putData(this)
    }
}