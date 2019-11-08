package frc.robot

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object Constants {

    const val kIsRocketLeague =
            true

    object DriveConstants {
        const val kBeta = 2.0 * 1.2
        const val kZeta = 0.6

        val kRobotMass = (50.0 /* Robot, kg */ + 5.0 /* Battery, kg */ + 2.0 /* Bumpers, kg */).toDouble()
        private val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Tune
        private val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

        private val kWheelRadius = (2.0).inch
        private val kTrackWidth = (26.0).inch

        val kDriveLengthModel = NativeUnitLengthModel(4096.nativeUnits, kWheelRadius)

        private val kVDriveLeftLow = 0.274 // Volts per radians per second
        private val kADriveLeftLow = 0.032 // Volts per radians per second per second
        private val kVInterceptLeftLow = 1.05 // Volts

        private val kVDriveRightLow = 0.265 // Volts per radians per second
        private val kADriveRightLow = 0.031 // Volts per radians per second per second
        private val kVInterceptRightLow = 1.02 // Volts

        private val kLeftTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveLeftLow,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveLeftLow),
                kVInterceptLeftLow)

        private val kRightTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveRightLow,
                kWheelRadius.meter * kWheelRadius.meter * kRobotMass / (2.0 * kADriveRightLow),
                kVInterceptRightLow)

        private val kVDriveLeftHigh = 0.115 * 1.0 // Volts per radians per second
        private val kADriveLeftHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptLeftHigh = 1.8 * 1.0 // 4 * 0.4d; // Volts

        private val kVDriveRightHigh = 0.11 * 1.0 // Volts per radians per second
        private val kADriveRightHigh = 0.043 * 1.0 // Volts per radians per second per second
        private val kVInterceptRightHigh = 1.75 * 1.0 // 4 * 0.4d; // Volts

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

    object SuperStructureConstants {
        val kProximalStatic = 0.4.volt // volts
        val kProximalCos = 0.94.volt // volts
        const val kJointSpeedMultiplier = 1.0
        val kProximalLen = 32.0.inch
        val kElevatorRange = 11.inch..69.inch
        val kProximalThrustVelocity = 50.degree.velocity
    }

    object IntakeConstants {

//        val deployTime = 0.1.second
    }

    /* Wrist stuff */
    val kWristLength = 6.inch // distance from joint to COM
    val kWristMass = 15.lb
    val kWristSpeedPerVolt = 0.21 // radians/sec/volt
    val kWristTorquePerVolt = 47.33 // Newton meters per volt, stall
    val kWristStaticFrictionVoltage = 0.0 // volts, TODO tune

    /* Elbow stuff */
    val kElbowLength = 8.inch // distance from joint to COM
    val kElbowMass = 3.lb
    val kElbowSpeedPerVolt = 0.17 // radians/sec/volt
    val kElbowTorquePerVolt = 55.0 // Newton meters per volt, stall
    val kElbowStaticFrictionVoltage = 0.0 // volts, TODO tune

    // ROBOT AND MECHANISM DIMENSIONS

    val kRobotWidth = 28.75.inch
    val kRobotLength = 31.inch

    val kBumperThickness = 3.25.inch
    val kCenterToElevator = (kRobotLength / 2) - 11.inch // 4.5
    val kBadIntakeOffset = 0.inch
    val kArmLength = 29.5.inch // from center of elevator to hatch part of the intake

    val kIntakeProtrusionFrontExtended = kArmLength - (kRobotLength / 2.0 - kCenterToElevator) // 18.5
    val kIntakeProtrusionBackExtended = kCenterToElevator - kArmLength + kRobotLength / 2.0 // -9.5

    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meter, 0.degree)

    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusionFrontExtended, kBadIntakeOffset, 0.degree) // -34
    val kForwardIntakeStowedToCenter = Pose2d(-(kRobotLength / 2.0) - 8.inch, kBadIntakeOffset, 0.degree) // -23.5
    val kForwardIntakePokedStowedToCenter = Pose2d(-(kRobotLength / 2.0) - 8.inch - 12.7.inch, kBadIntakeOffset, 0.degree) // -23.5 TODO change this for poked out
    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusionFrontExtended, -kBadIntakeOffset, 0.degree) // 34
    val kCenterToForwardIntakeStowed = Pose2d((kRobotLength / 2.0) + 8.inch, kBadIntakeOffset, 0.degree) // 23.5
    val kBackwardIntakeToCenter = Pose2d(kCenterToForwardIntake.translation.x - kCenterToElevator, -kBadIntakeOffset, 0.degree) // 29.5

    val kCenterToFrontCamera = Pose2d(kRobotLength / 2 - 13.inch, 0.0.inch, 0.degree)
    val kCenterToBackCamera = Pose2d(kRobotLength / 2 - 16.inch, 0.0.inch, 180.degree)
}
