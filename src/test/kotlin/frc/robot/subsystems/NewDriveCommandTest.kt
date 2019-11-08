package frc.robot.subsystems

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import frc.robot.subsystems.drive.times
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kBumperRight

object DriveConstants {
    const val kBeta = 2.0
    const val kZeta = 0.7

    val kRobotMass = (50.0 /* Robot, kg */ + 5.0 /* Battery, kg */ + 2.0 /* Bumpers, kg */).toDouble()
    private val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Tune
    private val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

    private val kWheelRadius = (2.0).inch
    private val kTrackWidth = (26.0).inch

    val kDriveLengthModel = NativeUnitLengthModel(4096.nativeUnits, kWheelRadius)

    private val kVDriveLeftLow = 0.274 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveLeftLow = 0.032 * 1.0 // Volts per radians per second per second TODO tune
    private val kVInterceptLeftLow = 1.05 * 1.0 // Volts - tuned!

    private val kVDriveRightLow = 0.265 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveRightLow = 0.031 * 1.0 // Volts per radians per second per second TODO tune
    private val kVInterceptRightLow = 1.02 * 1.0 // Volts - tuned!

    private val kLeftTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveLeftLow,
            kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveLeftLow),
            kVInterceptLeftLow)

    private val kRightTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveRightLow,
            kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveRightLow),
            kVInterceptRightLow)

    private val kVDriveLeftHigh = 0.143 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveLeftHigh = 0.043 * 1.0 // Volts per radians per second per second
    private val kVInterceptLeftHigh = 1.33 * 1.0 // 4 * 0.4d; // Volts - tuned!

    private val kVDriveRightHigh = 0.14 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveRightHigh = 0.043 * 1.0 // Volts per radians per second per second
    private val kVInterceptRightHigh = 1.34 * 1.0 // 4 * 0.4d; // Volts - tuned!

    private val kLeftTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveLeftHigh,
            kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveLeftHigh),
            kVInterceptLeftHigh)

    private val kRightTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveRightHigh,
            kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveRightHigh),
            kVInterceptRightHigh)

    val kLowGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
            kRobotAngularDrag, kWheelRadius.meter, kTrackWidth.meter / 2.0, kLeftTransmissionModelLowGear, kRightTransmissionModelLowGear)

    val kHighGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
            kRobotAngularDrag, kWheelRadius.meter, kTrackWidth.meter / 2.0, kLeftTransmissionModelHighGear, kRightTransmissionModelHighGear)
}

object ManualDriveCommand {
    /**
     * Curvature or cheezy drive control
     */
    @Suppress("ComplexMethod")
    internal fun curvatureDrive(
        linearPercent: Double,
        curvaturePercent: Double,
        isQuickTurn: Boolean
    ): DifferentialDrive.WheelState {
        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (linearPercent.absoluteValue < TankDriveSubsystem.kQuickStopThreshold) {
                quickStopAccumulator = (1 - TankDriveSubsystem.kQuickStopAlpha) * quickStopAccumulator +
                        TankDriveSubsystem.kQuickStopAlpha * curvaturePercent.coerceIn(-1.0, 1.0) * 2.0
            }
            overPower = true
            angularPower = curvaturePercent
        } else {
            overPower = false
            angularPower = linearPercent.absoluteValue * curvaturePercent - quickStopAccumulator

            when {
                quickStopAccumulator > 1 -> quickStopAccumulator -= 1.0
                quickStopAccumulator < -1 -> quickStopAccumulator += 1.0
                else -> quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = linearPercent + angularPower
        var rightMotorOutput = linearPercent - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }
                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }
                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }
                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = max(leftMotorOutput.absoluteValue, rightMotorOutput.absoluteValue)
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        return DifferentialDrive.WheelState(leftMotorOutput, rightMotorOutput)
    }

    private var quickStopAccumulator = 0.0
    private const val kQuickStopThreshold = TankDriveSubsystem.kQuickStopThreshold
    private const val kQuickStopAlpha = TankDriveSubsystem.kQuickStopAlpha
    const val kDeadband = 0.05
    val speedSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband) }
    val rotationSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight).withDeadband(kDeadband) }
    val quickTurnSource by lazy { Controls.driverFalconXbox.getRawButton(kBumperRight)/*.getRawButton(kX)8*/ }
}

fun main() {

//    val curvature = 1.0
//    val linear = 1.0
//    val isQuickTurn = false
//
//    val differentialDrive = DriveConstants.kHighGearDifferentialDrive
//    val wheelSpeeds = ManualDriveCommand.curvatureDrive(linear, curvature, isQuickTurn).times(
//            if(false) 8.0 * kFeetToMeter else 12.0 * kFeetToMeter
//    )
//    val feedForwards = differentialDrive.getVoltagesFromkV(wheelSpeeds)
//    println("velocities ${wheelSpeeds.left.meter.feet}/${wheelSpeeds.right.meter.feet} voltages ${feedForwards.left}/${feedForwards.right}")

    // KISS rate function
    fun kscalc(rcCommand: Double, rate: Double, rcCurve: Double, rcRate: Double): Double {
        return ((2000.0 * (1.0 / (1 - abs(((rcCommand * ((rcCommand * 1000) * (rcCommand * 1000) / 1000000)) * rcCurve + rcCommand * (1 - rcCurve)) * (rcRate / 10)) * rate))) * ((rcCommand * ((rcCommand * 1000) * (rcCommand * 1000) / 1000000)) * rcCurve + rcCommand * (1 - rcCurve)) * (rcRate / 10))
    }
}
