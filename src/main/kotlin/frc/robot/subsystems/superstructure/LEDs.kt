package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import edu.wpi.first.networktables.EntryNotification
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.team5940.pantry.lib.ConcurrentlyUpdatingComponent
import java.awt.Color
import java.util.function.Consumer

object LEDs: FalconSubsystem() {

    class LEDBlinkCommand(val color: Color, val duration: Double): FalconCommand(LEDs) {
        var wantsOn = false
        var lastChangeTime = -1.0

        override fun isFinished() = false

        override fun execute() {
            // check if we need to change
            if(lastChangeTime + duration < Timer.getFPGATimestamp()) {
                if(wantsOn) setColor(color) else setColor(Color.BLACK)
                lastChangeTime = Timer.getFPGATimestamp()
                wantsOn = !wantsOn
            }
        }
    }

    override fun lateInit() {
//        setColor(PURPLE)
//        setColor(Color.red)
        println("INITing LED BLINK COMMAND")
        setColor(Color.RED)
//        LEDBlinkCommand(PURPLE, 1.0).schedule()
    }

    private val canifier by lazy { Proximal.canifier }
    fun setColor(color: Color) {
        println("Setting color to r${color.red} r${color.green} b${color.blue}")
        canifier.setLEDOutput(color.red * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelB)
        canifier.setLEDOutput(color.green * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelA)
        canifier.setLEDOutput(color.blue * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelC)
    }
    val PURPLE = Color(128, 0, 128)
}