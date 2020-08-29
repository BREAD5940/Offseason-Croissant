package frc.robot

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
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

        val kWheelRadius = (2.0).inches
        val kTrackWidth = (26.0).inches

        val kinematics = DifferentialDriveKinematics(kTrackWidth.inMeters())

        val kDriveLengthModel = NativeUnitLengthModel(4096.nativeUnits, kWheelRadius)

        private val kVDriveLeftLow = 0.274 / kWheelRadius.inMeters() // Volts per meter per second
        private val kADriveLeftLow = 0.032 / kWheelRadius.inMeters() // Volts per meter per second per second
        private val kVInterceptLeftLow = 1.05 // Volts

        // pi * (2 * r) meters per rotation
        // = pi * 2 * r meters / rotations
        // = pi * 2 * r meters / (2pi radians / 1 rotation * rotations)
        // = r meters per radian
        // sooooo
        // 1 volts / (radians / seconds) = 1 volts / (r meters per radian times radians per second)
        // = 1 volt / (radius) / (meter / second)

        private val kVDriveRightLow = 0.265 / kWheelRadius.inMeters() // Volts per meter per second
        private val kADriveRightLow = 0.031 / kWheelRadius.inMeters() // Volts per meter per second per second
        private val kVInterceptRightLow = 1.02 // Volts

        val kLeftTransmissionModelLowGear = SimpleMotorFeedforward(
                kVDriveLeftLow, kADriveLeftLow, kVInterceptLeftLow
        )
            // DCMotorTransmission(1 / kVDriveLeftLow,
//                kWheelRadius.inMeters() * kWheelRadius.inMeters() * kRobotMass / (2.0 * kADriveLeftLow),
//                kVInterceptLeftLow)

        val kRightTransmissionModelLowGear = SimpleMotorFeedforward(
                kVDriveRightLow, kADriveLeftLow, kVInterceptRightLow
        )
            // DCMotorTransmission(1 / kVDriveRightLow,
//                kWheelRadius.inMeters() * kWheelRadius.inMeters() * kRobotMass / (2.0 * kADriveRightLow),
//                kVInterceptRightLow)

        private val kVDriveLeftHigh = 0.115 / kWheelRadius.inMeters() // Volts per meter per second
        private val kADriveLeftHigh = 0.043 / kWheelRadius.inMeters() // Volts per meter per second per second
        private val kVInterceptLeftHigh = 1.8 // 4 * 0.4d; // Volts

        private val kVDriveRightHigh = 0.11 / kWheelRadius.inMeters() // Volts per meter per second
        private val kADriveRightHigh = 0.043 / kWheelRadius.inMeters() // Volts per meter per second per second
        private val kVInterceptRightHigh = 1.75 // 4 * 0.4d; // Volts

        val kLeftTransmissionModelHighGear = SimpleMotorFeedforward(
                kVDriveLeftHigh, kADriveLeftHigh, kVInterceptLeftHigh
        )
            // DCMotorTransmission(1 / kVDriveLeftHigh,
//                kWheelRadius.inMeters() * kWheelRadius.inMeters() * kRobotMass / (2.0 * kADriveLeftHigh),
//                kVInterceptLeftHigh)

        val kRightTransmissionModelHighGear = SimpleMotorFeedforward(
                kVDriveRightHigh, kADriveRightHigh, kVInterceptRightHigh
        )
            // DCMotorTransmission(1 / kVDriveRightHigh,
//                kWheelRadius.inMeters() * kWheelRadius.inMeters() * kRobotMass / (2.0 * kADriveRightHigh),
//                kVInterceptRightHigh)

//        val kLowGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
//                kRobotAngularDrag, kWheelRadius.inMeters(), kTrackWidth.inMeters() / 2.0, kLeftTransmissionModelLowGear, kRightTransmissionModelLowGear)
//
//        val kHighGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
//                kRobotAngularDrag, kWheelRadius.inMeters(), kTrackWidth.inMeters() / 2.0, kLeftTransmissionModelHighGear, kRightTransmissionModelHighGear)
    }

    object SuperStructureConstants {
        val kProximalStatic = 0.4.volts // volts
        val kProximalCos = 0.94.volts // volts
        const val kJointSpeedMultiplier = 1.0
        val kProximalLen = 32.0.inches
        val kElevatorRange = 11.inches..69.inches
        val kProximalThrustVelocity = 50.degrees.velocity
    }

    object IntakeConstants {

//        val deployTime = 0.1.second
    }

    /* Wrist stuff */
    val kWristLength = 6.inches // distance from joint to COM
    val kWristMass = 15.lb
    val kWristSpeedPerVolt = 0.21 // radians/sec/volt
    val kWristTorquePerVolt = 47.33 // Newton meters per volt, stall
    val kWristStaticFrictionVoltage = 0.0 // volts, TODO tune

    /* Elbow stuff */
    val kElbowLength = 8.inches // distance from joint to COM
    val kElbowMass = 3.lb
    val kElbowSpeedPerVolt = 0.17 // radians/sec/volt
    val kElbowTorquePerVolt = 55.0 // Newton meters per volt, stall
    val kElbowStaticFrictionVoltage = 0.0 // volts, TODO tune

    // ROBOT AND MECHANISM DIMENSIONS

    val kRobotWidth = 28.75.inches
    val kRobotLength = 31.inches

    val kBumperThickness = 3.25.inches
    val kCenterToElevator = (kRobotLength / 2) - 11.inches // 4.5
    val kBadIntakeOffset = 0.inches
    val kArmLength = 29.5.inches // from center of elevator to hatch part of the intake

    val kIntakeProtrusionFrontExtended = kArmLength - (kRobotLength / 2.0 - kCenterToElevator) // 18.5
    val kIntakeProtrusionBackExtended = kCenterToElevator - kArmLength + kRobotLength / 2.0 // -9.5

    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meters, 0.degrees)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meters, 0.degrees)

    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusionFrontExtended, kBadIntakeOffset, 0.degrees) // -34
    val kForwardIntakeStowedToCenter = Pose2d(-(kRobotLength / 2.0) - 8.inches, kBadIntakeOffset, 0.degrees) // -23.5
    val kForwardIntakePokedStowedToCenter = Pose2d(-(kRobotLength / 2.0) - 8.inches - 12.7.inches, kBadIntakeOffset, 0.degrees) // -23.5 TODO change this for poked out
    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusionFrontExtended, -kBadIntakeOffset, 0.degrees) // 34
    val kCenterToForwardIntakeStowed = Pose2d((kRobotLength / 2.0) + 8.inches, kBadIntakeOffset, 0.degrees) // 23.5
    val kBackwardIntakeToCenter = Pose2d(kCenterToForwardIntake.translation.x.meters - kCenterToElevator, -kBadIntakeOffset, 0.degrees) // 29.5

    val kCenterToFrontCamera = Pose2d(kRobotLength / 2 - 13.inches, 0.0.inches, 0.degrees)
    val kCenterToBackCamera = Pose2d(kRobotLength / 2 - 16.inches, 0.0.inches, 180.degrees)
}

//private fun <T: SIKey> MotorCharacterization(kV: Double, kA: Double, kS: Double) =
//        MotorCharacterization<T>(
//                SIUnit(kV),
//                SIUnit(kA),
//                SIUnit(kS)
//        )

//fun MotorCharacterization<*>.toSimpleFeedforward() = SimpleMotorFeedforward(kS.value, kV.value, kA.value)