// package frc.robot.subsystems
//
// import com.team254.lib.physics.DCMotorTransmission
// import edu.first.wpilibj.trajectory.TrapezoidProfile
// import frc.robot.Constants
// import org.ghrobotics.lib.mathematics.units.SIUnit
// import org.ghrobotics.lib.mathematics.units.derived.Volt
// import org.ghrobotics.lib.mathematics.units.derived.volt
// import org.ghrobotics.lib.mathematics.units.inch
// import org.ghrobotics.lib.mathematics.units.kInchToMeter
// import org.ghrobotics.lib.mathematics.units.meter
// import org.knowm.xchart.SwingWrapper
// import org.knowm.xchart.XYChart
// import org.knowm.xchart.XYChartBuilder
// import kotlin.math.PI
//
// fun main() {
//
//    fun getVoltage(state: TrapezoidProfile.State): SIUnit<Volt> {
//        val linearSpeed = state.velocity
//        // meters per second div meters per rotation is rotations per second
//        val rotPerSec = linearSpeed / (PI * 1.5.inch.meter)
//        val radPerSec = rotPerSec * PI * 2
//
//        // torque is force times distance so
//        val torque = 35.0 /* kg */ * 9.8 /* g */ * 0.75.inch.meter
// //        println("torque $torque")
// //        println("angular Velocity $radPerSec")
//
//        // stall torque of the neo is 2.6 newton meters
//        // at a 42 to 1 the stall torque is 42 * 2.6
// //        val stallTorque = 42.0 * 2.6
// //        val freeYeet = 594.4 /* rad per sec */ / 42.0
//        val stallTorque = 14.66 * 0.71 * 4
//        val freeYeet = 1961 /* rad per sec */ / 42.0
//        val voltage = torque / stallTorque + radPerSec / freeYeet
//
//        return voltage.volt
//    }
//
//    val profile = TrapezoidProfile(TrapezoidProfile.Constraints(0.3, 3.0), // meters per sec and meters per sec ^2
//            TrapezoidProfile.State(12.inch.meter, 0.0), TrapezoidProfile.State(25.inch.meter, 0.0)
//    )
//    val x: ArrayList<Double> = arrayListOf()
//    val pos: ArrayList<Double> = arrayListOf()
//    val vel: ArrayList<Double> = arrayListOf()
//    var currentTime = 0.0
//    while(!profile.isFinished(currentTime)) {
//        currentTime += 1.0 / 50.0
//        x.add(currentTime)
//        pos.add(profile.calculate(currentTime).position)
//        vel.add(profile.calculate(currentTime).velocity)
//
//    }
//
//    val chart = XYChart(XYChartBuilder())
//    chart.addSeries("pos", x, pos)
//    chart.addSeries("vel", x, vel)
//
//    (SwingWrapper(chart)).displayChart()
//
//
// }
