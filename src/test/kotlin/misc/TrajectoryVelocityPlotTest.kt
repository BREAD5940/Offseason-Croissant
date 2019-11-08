package misc

import com.team254.lib.physics.DifferentialDrive
import frc.robot.Constants
import frc.robot.auto.paths.TrajectoryFactory
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat
import kotlin.math.max
import kotlin.math.min
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derived.feetPerSecond
import org.ghrobotics.lib.mathematics.units.milli
import org.ghrobotics.lib.mathematics.units.second
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder

class TrajectoryVelocityPlotTest {

    @Test
    fun test() {
//        val (x1, y1) = trajectToData(TrajectoryFactory.sideStartReversedToRocketFPrepare)
        val (x1, y1, z1) = trajectToData(TrajectoryFactory.rocketFPrepareToLoadingStation)

        displayTrajectory(Triple(x1, y1, z1))
    }

    private fun trajectToData(trajectory: TimedTrajectory<Pose2dWithCurvature>): Triple<ArrayList<Double>, ArrayList<Double>, ArrayList<Double>> {

        var currentTime = 0.second
        val deltaTime = 20.milli.second

        val xList = arrayListOf<Double>()
        val yList = arrayListOf<Double>()
        val zList = arrayListOf<Double>()
        val iterator = trajectory.iterator()
        while (!iterator.isDone) {
            currentTime += deltaTime
            val sample = iterator.advance(deltaTime).state
            val velocity = sample.velocity
            xList += currentTime.second
            yList += velocity.feetPerSecond

            val vd = sample.velocity.value
            val wd = vd * sample.state.curvature

            val voltage = Constants.DriveConstants.kHighGearDifferentialDrive.getVoltagesFromkV(
                    Constants.DriveConstants.kHighGearDifferentialDrive.solveInverseKinematics(DifferentialDrive.ChassisState(vd, wd))
            ).let { max(it.left, it.right) }
            zList += voltage
        }

        return Triple(xList, yList, zList)
    }

    private fun displayTrajectory(data: Triple<ArrayList<Double>, ArrayList<Double>, ArrayList<Double>>) {

        val xList = data.first
        val yList = data.second
        val zList = data.third

        val fm = DecimalFormat("#.###").format(xList.max()!!)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
                .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = xList.min()!!
        chart.styler.xAxisMax = xList.max()!!
        chart.styler.yAxisMin = min(yList.min()!!, zList.min()!!)
        chart.styler.yAxisMax = max(yList.max()!!, zList.max()!!)

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Robot velocity", xList.toDoubleArray(), yList.toDoubleArray())
        chart.addSeries("Voltage", xList.toDoubleArray(), zList.toDoubleArray())

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }
}
