
package frc.robot

import com.ctre.phoenix.CANifier
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.frc2.command.CommandScheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.auto.Autonomous
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Proximal
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
import frc.robot.vision.JeVoisManager
import frc.robot.vision.LimeLightManager
import frc.robot.vision.TargetTracker
import frc.robot.vision.VisionProcessing
import frc.robot.subsystems.superstructure.Wrist
import org.ghrobotics.lib.mathematics.units.inch
import org.team5940.pantry.lib.FishyRobot
import setLEDOutput
import java.awt.Color
import edu.wpi.first.wpilibj.networktables.NetworkTable.getTable
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber



object Robot : FishyRobot() {

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated

        +DriveSubsystem
        +Proximal
        +Wrist
        +Elevator
        +Superstructure
        +Intake
        +ClimbSubsystem

        +TargetTracker
        JeVoisManager
        LimeLightManager
        VisionProcessing
        +Controls
        +Autonomous
        +LEDs

        SmartDashboard.putData(CommandScheduler.getInstance())
        Superstructure.zero.schedule()

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2)

        super.robotInit()
    }

    override fun robotPeriodic() {
        super.robotPeriodic()
    }

    override fun autonomousInit() {
        super.autonomousInit()
    }

}

fun main() {
    Robot.start()
}