package frc.robot.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Constants
import frc.robot.Controls
import kotlin.math.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports

open class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    override fun initialize() {
        DriveSubsystem.run {
            listOf(leftMotor.master, rightMotor.master).forEach {
                it.brakeMode = true
                it.talonSRX.configOpenloopRamp(0.16)
                it.talonSRX.configClosedloopRamp(0.00)
            }
        }
    }

    override fun execute() {
        val curvature = rotationSource()
        val linear = -speedSource()
        val speedMultiplier = reduceSpeedSource() // 0 to 1
        val isQuickTurn = quickTurnSource() || linear.absoluteValue < 0.25 || speedMultiplier > 0.5

        DriveSubsystem.curvatureDrive(
                linear * linear.absoluteValue * 0.9 * (1 - speedMultiplier * 0.3),
                curvature * curvature.absoluteValue * 0.8 * (1 - speedMultiplier * 0.35) * if (isQuickTurn) 0.7 else 1.0,
                isQuickTurn)

//        curvatureDrive(
//                linear,
//                driveCubicDeadband,
//                quickTurnSource())
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.run {
            listOf(leftMotor.master, rightMotor.master).forEach {
                it.brakeMode = true
                it.talonSRX.configOpenloopRamp(0.0)
            }
        }
    }

    /**
     * Tank drive control
     */
    private fun tankDrive(
        leftPercent: Double,
        rightPercent: Double
    ) {
        DriveSubsystem.leftMotor.setDutyCycle(leftPercent)
        DriveSubsystem.rightMotor.setDutyCycle(rightPercent)
    }

    companion object {
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

        // KISS rate function
        fun kisscalc(rcCommand: Double, rate: Double, rcCurve: Double, rcRate: Double): Double {
            val kissRpyUseRates = 1 - Math.abs(rcCommand) * rate
            val kissRxRaw = rcCommand * 1000
            val kissTempCurve = (kissRxRaw * kissRxRaw / 1000000)
            val rcCommand_ = ((rcCommand * kissTempCurve) * rcCurve + rcCommand * (1 - rcCurve)) * (rcRate / 10)
            return ((2000.0 * (1.0 / kissRpyUseRates)) * rcCommand_)
        }

        private var quickStopAccumulator = 0.0
        private const val kQuickStopThreshold = TankDriveSubsystem.kQuickStopThreshold
        private const val kQuickStopAlpha = TankDriveSubsystem.kQuickStopAlpha
        const val kDeadband = 0.05
        val speedSource: () -> Double by lazy {
            if (Constants.kIsRocketLeague) {
                return@lazy { val toRet = Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kRight) - Controls
                        .driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft)
//                    println("speed $toRet")
                    val compensated = toRet * -1.0
                    val deadbanded = ((compensated.absoluteValue - kDeadband / 1.8) / (1.0 - kDeadband / 1.8)) * compensated.sign

                    // throttle curve should be between [-1. 1] or so.
                    kisscalc(deadbanded, 0.77, 0.0, 0.00116)
                }
            } else {
                return@lazy Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
            }
        }
//            if(Constants.kIsRocketLeague) { { Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kRight)
//                - Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft) } }
//            else Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)

        val rotationSource by lazy {
            if (Constants.kIsRocketLeague) Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
            else Controls.driverFalconXbox.getX(GenericHID.Hand.kRight).withDeadband(kDeadband)
        }
        val quickTurnSource by lazy {
            if (Constants.kIsRocketLeague) Controls.driverFalconXbox.getRawButton(kA)
            else Controls.driverFalconXbox.getRawButton(kBumperRight)
        }
        val reduceSpeedSource by lazy { { Controls.driverControllerLowLevel.getTriggerAxis(GenericHID.Hand.kLeft) } }
    }
}
