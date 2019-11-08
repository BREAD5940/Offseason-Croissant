/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package frc.robot.subsystems.drive

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import kotlin.math.PI
import org.ghrobotics.lib.commands.FalconCommand

/**
 * Command that characterizes the robot using the robotpy-characterization
 * toolsuite.
 *
 * @param drivetrain The instance of FalconWCD to use.
 */
class CharacterizationCommand(private val drivetrain: DriveSubsystem) : FalconCommand(drivetrain) {

    private val numberArray = DoubleArray(9)

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    override fun runsWhenDisabled(): Boolean {
        return true
    }

    private var priorAutoSpeed = 0.0

    override fun execute() {
        val autospeed = autoSpeedEntry.getDouble(0.0)
        priorAutoSpeed = autospeed

        drivetrain.tankDrive(autospeed, autospeed)

        numberArray[0] = Timer.getFPGATimestamp()
        numberArray[1] = RobotController.getBatteryVoltage()
        numberArray[2] = autospeed
        numberArray[3] = drivetrain.leftMotor.voltageOutput.value
        numberArray[4] = drivetrain.rightMotor.voltageOutput.value
        numberArray[5] = DriveSubsystem.leftMotor.master.talonSRX.selectedSensorPosition.toDouble() / 4096.0 * 2 * PI // encoder.position.value
        numberArray[6] = DriveSubsystem.rightMotor.master.talonSRX.selectedSensorPosition.toDouble() / 4096.0 * 2 * PI
        numberArray[7] = DriveSubsystem.leftMotor.master.talonSRX.selectedSensorVelocity.toDouble() / 4096.0 * 10.0 * 2 * PI
        numberArray[8] = DriveSubsystem.rightMotor.master.talonSRX.selectedSensorVelocity.toDouble() / 4096.0 * 10.0 * 2 * PI

        telemetryEntry.setNumberArray(numberArray.toTypedArray())
    }

    override fun end(interrupted: Boolean) {
        drivetrain.setNeutral()
    }
}
