package misc

import frc.robot.auto.paths.TrajectoryFactory
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Twist2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import org.team5940.pantry.lib.defaultLTVUnicycleController

class TrackerTest {

    class SimDiffDrive {
        var robotLocation = Pose2d(0.meter, 0.meter, 0.radian)

        fun update(deltaTime: SIUnit<Second>, wheelSpeeds: TrajectoryTrackerOutput) {
            val linear = wheelSpeeds.linearVelocity.coerceIn(-4.0.feet.velocity, 4.0.feet.velocity)

            val twist = Twist2d(
                    SIUnit(linear.value * deltaTime.second),
                    0.meter,
                    SIUnit(wheelSpeeds.angularVelocity.value * deltaTime.second)
            )

            val newPose = robotLocation + twist.asPose

            robotLocation = newPose
        }
    }

    @Test
    fun testLTVUnicycleController() {

        val trajectory = TrajectoryFactory.testTrajectory2

        val controller = defaultLTVUnicycleController // RamseteController(2.0, 0.7) //defaultLTVUnicycleController
        val drive = SimDiffDrive()

        drive.robotLocation = trajectory.firstState.state.pose // Pose2d(4.feet, 4.feet, 45.degree)

        var currentTime = 0.second
        val deltaTime = 20.milli.second

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()

        val refXList = arrayListOf<Double>()
        val refYList = arrayListOf<Double>()

        controller.reset(trajectory)

        while (currentTime.second < trajectory.duration.second) {

            val nextTime = currentTime + deltaTime
            val state = trajectory.sample(nextTime)

            val output = controller.nextState(drive.robotLocation, currentTime)

            drive.update(deltaTime, output)

            val newX = drive.robotLocation.translation.x
            val newY = drive.robotLocation.translation.y

            xList += newX.feet
            yList += newY.feet

            val wantedX = state.state.state.pose.translation.x
            val wantedY = state.state.state.pose.translation.y

            refXList += wantedX.feet
            refYList += wantedY.feet

            currentTime = nextTime
        }

        val fm = DecimalFormat("#.###").format(trajectory.duration.second)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
                .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = -1.0
        chart.styler.xAxisMax = refXList.max()
        chart.styler.yAxisMin = -1.0
        chart.styler.yAxisMax = refXList.max()

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Trajectory", refXList.toDoubleArray(), refYList.toDoubleArray())
        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

//        val terror =
//            TrajectoryGeneratorTest.trajectory.lastState.state.pose.translation - drive.robotLocation.translation
//        val rerror = TrajectoryGeneratorTest.trajectory.lastState.state.pose.rotation - drive.robotLocation.rotation

//        System.out.printf("%n[Test] X Error: %3.3f, Y Error: %3.3f%n", terror.x.feet, terror.y.feet)

//        assert(terror.norm.value.also {
//            println("[Test] Norm of Translational Error: $it")
//        } < 0.50)
//        assert(rerror.degree.also {
//            println("[Test] Rotational Error: $it degrees")
//        } < 5.0)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }
}
