package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.ghrobotics.lib.localization.Localization
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.wrappers.FalconMotor
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.SPI
import frc.robot.Constants
import org.ghrobotics.lib.mathematics.units.degree
import frc.robot.Ports
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase
import org.ghrobotics.lib.wrappers.LinearFalconMotor
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import kotlin.properties.Delegates



class Drive(
        val left:  FalconMotor<Length>,
        val right: FalconMotor<Length>,
        val shifter: DoubleSolenoid,
        val gyro: AHRS,
        val localization: Localization
            ) : DifferentialTrackerDriveBase {

    var currentTrajectoryTracker : TrajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)

    init {
//        trajectoryTracker = RamseteTracker(Constants.DriveConstants.kBeta, Constants.DriveConstants.kZeta)
        localization.reset(Pose2d())
        Notifier{
            localization.update()
        }.startPeriodic(1.0 / 100.0)
    }

    // Shift up and down
    var lowGear : Boolean by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.set(kForward)
        } else {
            shifter.set(kReverse)
        }
    }

    override val leftMotor: LinearFalconMotor
        get() = left

    override val rightMotor: LinearFalconMotor
        get() = right

    override val trajectoryTracker: TrajectoryTracker
        get() = currentTrajectoryTracker

    override val differentialDrive: DifferentialDrive
        get() = if(lowGear) Constants.DriveConstants.kLowGearDifferentialDrive else Constants.DriveConstants.kHighGearDifferentialDrive

    override val robotPosition: Pose2d
        get() = localization.robotPosition

    companion object {
        fun createNewTalonDrive() : Drive {
            val leftMotors = listOf(
                    FalconSRX<Length>(Ports.DrivePorts.LEFT_PORTS[0], Constants.DriveConstants.kDriveLengthModel),
                    FalconSRX<Length>(Ports.DrivePorts.LEFT_PORTS[1], Constants.DriveConstants.kDriveLengthModel)
            )

            val rightMotors = listOf(
                    FalconSRX<Length>(Ports.DrivePorts.RIGHT_PORTS[0], Constants.DriveConstants.kDriveLengthModel),
                    FalconSRX<Length>(Ports.DrivePorts.RIGHT_PORTS[1], Constants.DriveConstants.kDriveLengthModel)
            )

            leftMotors.forEach{
                it.setInverted(true)
            }

            val leftTransmission = Transmission(leftMotors)
            val rightTransmission = Transmission(rightMotors)

            val shifter = DoubleSolenoid(Ports.kPCMID, Ports.DrivePorts.SHIFTER_PORTS[0], Ports.DrivePorts.SHIFTER_PORTS[1])

            val gyro = AHRS(SPI.Port.kMXP)

            val localization = TankEncoderLocalization(
                    {(gyro.getFusedHeading() * -1).degree},
                    {leftMotors[0].sensorPosition.meter},
                    {rightMotors[0].sensorPosition.meter}
            )

            return Drive(
                    leftTransmission,
                    rightTransmission,
                    shifter,
                    gyro,
                    localization
            )

        }
    }

}








