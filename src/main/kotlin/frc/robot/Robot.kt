
package frc.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.auto.Autonomous
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Proximal
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import frc.robot.vision.JeVoisManager
import frc.robot.vision.TargetTracker
import frc.robot.vision.VisionProcessing
import frc.robot.subsystems.superstructure.Wrist
import org.team5940.pantry.lib.FishyRobot

object Robot : FishyRobot() {

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated

        +DriveSubsystem
        +Proximal
        +Wrist
        +Elevator
        +Superstructure
        +IntakeSubsystem
        +ClimbSubsystem

        +TargetTracker
        JeVoisManager
//        LimeLightManager
        VisionProcessing
        +Controls
        +Autonomous
        +LEDs

        SmartDashboard.putData(CommandScheduler.getInstance())
        Superstructure.zero.schedule()

//        LimeLight.lateinit()
        LimeLight.configureDisabled()

        super.robotInit()
    }

    override fun autonomousInit() {
        LimeLight.configureEnabled()
    }

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
        super.robotPeriodic()
    }

    override fun disabledInit() {
        LimeLight.configureDisabled()
    }

    override fun teleopInit() {
        LimeLight.configureEnabled()
        DriveSubsystem.lowGear = false
    }
}

fun main() {
    Robot.start()
}