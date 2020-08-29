package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.Constants.DriveConstants.kDriveLengthModel
import frc.robot.Ports.DrivePorts.LEFT_PORTS
import frc.robot.Ports.DrivePorts.RIGHT_PORTS
import frc.robot.Ports.DrivePorts.SHIFTER_PORTS
import frc.robot.Ports.kPCMID
import frc.robot.auto.paths.Pose2d
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.asSource
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import org.team5940.pantry.lib.MultiMotorTransmission

object DriveSubsystem : FalconWestCoastDrivetrain(), SensorlessCompatibleSubsystem, ConcurrentlyUpdatingComponent {

    public override val leftMotor: MultiMotorTransmission<Meter, FalconSRX<Meter>> = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master = FalconSRX(LEFT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(LEFT_PORTS[1], DefaultNativeUnitModel))

        init {
            master.talonSRX.configFactoryDefault(50)
            followers[0].talonSRX.configFactoryDefault(50)
            master.talonSRX.configClosedLoopPeriod(0, 10)
            outputInverted = true
            followers.forEach { it.follow(master) }
            lateInit()
            master.talonSRX.configContinuousCurrentLimit(38)
            master.talonSRX.configPeakCurrentDuration(500)
            master.talonSRX.configPeakCurrentLimit(60)
            followers[0].talonSRX.configContinuousCurrentLimit(38)
            followers[0].talonSRX.configPeakCurrentDuration(500)
            followers[0].talonSRX.configPeakCurrentLimit(60)
            master.talonSRX.enableCurrentLimit(true)
            followers[0].talonSRX.enableCurrentLimit(true)
        }


        override fun setClosedLoopGains() {
//            followers.forEach { configCurrentLimit(true, FalconSRX.CurrentLimitConfig(50.amp, 1.second, 38.amp)) }

            with(master.talonSRX) {
                configClosedLoopPeakOutput(0, 1.0)
                configPeakOutputForward(1.0)
                configPeakOutputReverse(-1.0)
            }

            // LQR gains
            if (lowGear) setClosedLoopGains(0.667, 0.0) else setClosedLoopGains(1.0 / 5.0, 0.0)
            // old gains
//            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.0, 0.0)
        }

        override var softLimitForward: SIUnit<Meter>
            get() = master.softLimitForward
            set(value) {
                master.softLimitForward = value
            }

        override var softLimitReverse: SIUnit<Meter>
            get() = master.softLimitReverse
            set(value) {
                master.softLimitReverse = value
            }
    }

    public override val rightMotor: MultiMotorTransmission<Meter, FalconSRX<Meter>> = object : MultiMotorTransmission<Meter, FalconSRX<Meter>>() {

        override val master = FalconSRX(RIGHT_PORTS[0], kDriveLengthModel)
        override val followers = listOf(FalconSRX(RIGHT_PORTS[1], DefaultNativeUnitModel))

        init {
            master.talonSRX.configFactoryDefault(50)
            followers[0].talonSRX.configFactoryDefault(50)
            followers.forEach { it.follow(master) }
            master.talonSRX.configClosedLoopPeriod(0, 10)
            lateInit()
            master.talonSRX.configContinuousCurrentLimit(38)
            master.talonSRX.configPeakCurrentDuration(500)
            master.talonSRX.configPeakCurrentLimit(60)
            followers[0].talonSRX.configContinuousCurrentLimit(38)
            followers[0].talonSRX.configPeakCurrentDuration(500)
            followers[0].talonSRX.configPeakCurrentLimit(60)
            master.talonSRX.enableCurrentLimit(true)
            followers[0].talonSRX.enableCurrentLimit(true)
        }

        override fun setClosedLoopGains() {
            // LQR gains
            if (lowGear) setClosedLoopGains(0.667, 0.0) else setClosedLoopGains(1.0 / 5.0, 0.0)

            with(master.talonSRX) {
                configClosedLoopPeakOutput(0, 1.0)
                configPeakOutputForward(1.0)
                configPeakOutputReverse(-1.0)
            }

            // Old gains
//            if (lowGear) setClosedLoopGains(0.45, 0.45*20.0) else setClosedLoopGains(1.0, 0.0)
        }

        override var softLimitForward: SIUnit<Meter>
            get() = master.softLimitForward
            set(value) {
                master.softLimitForward = value
            }

        override var softLimitReverse: SIUnit<Meter>
            get() = master.softLimitReverse
            set(value) {
                master.softLimitReverse = value
            }
    }

    override fun disableClosedLoopControl() {
        setNeutral(); leftMotor.zeroClosedLoopGains(); rightMotor.zeroClosedLoopGains()
        defaultCommand = ManualDriveCommand()
    }

    override fun enableClosedLoopControl() {
        leftMotor.setClosedLoopGains(); rightMotor.setClosedLoopGains()
        defaultCommand = ClosedLoopChezyDriveCommand()
    }

    fun notWithinRegion(region: Rectangle2d) =
            WaitUntilCommand { !region.contains(robotPosition.translation) }

    // Shift up and down
    val compressor = Compressor(9)
    private val shifter = FalconDoubleSolenoid(SHIFTER_PORTS[0], SHIFTER_PORTS[1], kPCMID)
    var lowGear: Boolean by Delegates.observable(false) { _, _, wantsLow ->

        shifter.state = if (wantsLow) FalconSolenoid.State.Reverse else FalconSolenoid.State.Forward

        // update PID gains
        leftMotor.setClosedLoopGains()
        rightMotor.setClosedLoopGains()
    }

    class SetGearCommand(wantsLow: Boolean) : InstantCommand(Runnable { lowGear = wantsLow }, this)

    private val ahrs = AHRS(SPI.Port.kMXP)
//    override val localization = TankEncoderLocalization(
//            ahrs.asSource(),
//            { leftMotor.encoder.position },
//            { rightMotor.encoder.position })

    override val gyro = ahrs.asSource()

    override val leftCharacterization: SimpleMotorFeedforward
        get() = if (lowGear) Constants.DriveConstants.kLeftTransmissionModelLowGear else Constants.DriveConstants.kLeftTransmissionModelHighGear

    override val rightCharacterization: SimpleMotorFeedforward
        get() = if (lowGear) Constants.DriveConstants.kRightTransmissionModelLowGear else Constants.DriveConstants.kRightTransmissionModelHighGear

    override val kinematics = Constants.DriveConstants.kinematics

    override val odometry = DifferentialDriveOdometry(gyro())

    // init localization stuff
    override fun lateInit() {
        // set the robot pose to a sane position
        odometry.resetPosition(Pose2d(20.feet, 20.feet, 0.degrees), gyro())
//        defaultCommand = ManualDriveCommand() // set default command
        defaultCommand = ClosedLoopChezyDriveCommand()
        super.lateInit()
    }

    // Ramsete gang is the only true gang
    override var controller = RamseteController(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    // the "differential drive" model, with a custom getter which changes based on the current gear
//    override val differentialDrive: DifferentialDrive
//        get() = if (lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    override fun updateState() {
//        localization.update()
    }

    fun setWheelVelocities(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        val leftFF = leftCharacterization.calculate(
                wheelSpeeds.leftMetersPerSecond, 0.0
        ).volts

        val rightFF = rightCharacterization.calculate(
                wheelSpeeds.rightMetersPerSecond, 0.0
        ).volts

        leftMotor.setVelocity(wheelSpeeds.leftMetersPerSecond.meters.velocity, leftFF)
        rightMotor.setVelocity(wheelSpeeds.rightMetersPerSecond.meters.velocity, rightFF)
    }

    fun setTrajectoryOutput(
            linearVelocity: SIUnit<LinearVelocity>,
            lienarAcceleration: SIUnit<LinearAcceleration>,
            angularVelocity: SIUnit<AngularVelocity>,
            angularAcceleration: SIUnit<AngularAcceleration>
    ) {
    }
}
