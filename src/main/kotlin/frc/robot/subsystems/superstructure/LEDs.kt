package frc.robot.subsystems.superstructure

import com.ctre.phoenix.CANifier
import java.awt.Color
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second

object LEDs : FalconSubsystem() {

    override fun lateInit() {
        updateThread.start()
    }

    fun setVisionMode(wantsVision: Boolean) {
        this.wantedState = if (wantsVision) {
            State.Blink((1.0 / 8.0).second, Color.red)
        } else {
            State.Default
        }
    }

    sealed class State(open val color: Color) {
        open class Solid(override val color: Color) : State(color)
        object Default : Solid(Color.red)
        object Off : Solid(Color.black)
        class Blink(val blinkTime: SIUnit<Second>, override val color: Color) : State(color)
//        class Fade(val fadeTime: SIUnit<Second>, override val color: Color): State(color)
    }

    var wantedState: State = State.Default
        @Synchronized get
        @Synchronized set

//    var lastWantedState = wantedState
//        @Synchronized get
//        @Synchronized set

    private val updateThread = Thread {
        while (true) {
            when (val wantedState = this@LEDs.wantedState) {
                is State.Solid -> { setColor(wantedState.color); Thread.sleep(250) }
                is State.Blink -> {
                    setColor(wantedState.color)
                    Thread.sleep(wantedState.blinkTime.millisecond.toLong() / 2)
                    setColor(Color.BLACK)
                    Thread.sleep(wantedState.blinkTime.millisecond.toLong() / 2)
                }
//                is State.Fade -> {
//                    val startColor = lastWantedState.color
//                    val endColor = wantedState.color
//                    val delta = endColor - startColor
//                    // we can do a subdivision every, say, 20ms
//                    // so we divide the total duration by 20ms to get how long we have
//                    val steps = (wantedState.fadeTime.millisecond / 20.0).toInt()
//                    for(i in 0..steps) {
//                        val interpolated = startColor + delta * (i.toDouble() / steps.toDouble())
//                        setColor(interpolated)
//                        delay(20)
//                    }
//                    setColor(endColor)
//                }
            }
//            lastWantedState = wantedState
        }
    }

    operator fun Color.plus(other: Color) = Color(this.red + other.red, this.green + other.green, this.blue + other.blue)
    operator fun Color.times(scalar: Double) = Color((this.red * scalar).toFloat(), (this.green * scalar).toFloat(), (this.blue * scalar).toFloat())
    operator fun Color.minus(other: Color) = Color(this.red - other.red, this.green - other.green, this.blue - other.blue)

    private val canifier by lazy { Proximal.canifier }
    fun setColor(color: Color) {
//        println("Setting color to r${color.red} r${color.green} b${color.blue}")
        canifier.setLEDOutput(color.red * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelB)
        canifier.setLEDOutput(color.green * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelA)
        canifier.setLEDOutput(color.blue * (1.0 / 255.0), CANifier.LEDChannel.LEDChannelC)
    }

    val PURPLE = Color(100, 0, 150)
}
