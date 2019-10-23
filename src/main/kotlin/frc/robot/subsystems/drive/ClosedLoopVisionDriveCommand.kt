package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.networktables.NetworkTableInstance
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

class ClosedLoopVisionDriveCommand(private val isFront: Boolean) : FalconCommand(DriveSubsystem) {

    override fun isFinished() = false

    private var referencePose = Pose2d()

    private var prevError = 0.0

    override fun initialize() {
        isActive = true
        referencePose = DriveSubsystem.robotPosition
        LEDs.wantedState = LEDs.State.Off
    }

    override fun execute() {

        var linear = -ManualDriveCommand.speedSource()

        if (!LimeLight.hasTarget) {
            super.execute()
        } else {

            val angle = LimeLight.lastYaw

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

            var offset = 0.degree
            var skew = LimeLight.lastSkew
            if(skew > (-45).degree) skew = skew.absoluteValue else skew += 90.degree
            if(skew > 5.degree) offset = 0.05.degree * (if (LimeLight.targetToTheLeft) 1 else -1) * (skew.degree / 13);

            val error = angle.radian - offset.radian

            var turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            if(turn.absoluteValue > maxTurn.value) turn = maxTurn.value

            DriveSubsystem.setWheelVelocities(DifferentialDrive.WheelState((linear - turn) * multiplier, (linear + turn) * multiplier))

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
        var kCorrectionKp = 1.0
        var kCorrectionKd = 0.0
        val maxTurn = 30.degree.velocity
        var isActive = false
            private set
    }

    init {
        SmartDashboard.putData(this)
    }
}