// package misc
//
// import edu.wpi.first.wpilibj.controller.RamseteController
// import edu.wpi.first.wpilibj.geometry.Twist2d
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
// import java.awt.Color
// import java.awt.Font
// import java.text.DecimalFormat
// import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
// import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
// import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
// import org.junit.Test
// import org.knowm.xchart.SwingWrapper
// import org.knowm.xchart.XYChartBuilder
// import frc.robot.auto.paths.TrajectoryFactory
// import generated.kotlin.statespace.generatedLTVUnicycleController
// import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
// import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryGenerator
//
// class TrackerTest {
//
//    @Test
//    fun testLTVUnicycleController() {
//
//        val yes = 42.0
//
//        val trajectory = FalconTrajectoryGenerator.generateTrajectory(
//                listOf(Pose2d(2.feet, 2.feet, 0.degrees.toRotation2d()),
//                Pose2d(10.feet, 10.feet, 0.degrees.toRotation2d())),
//                FalconTrajectoryConfig(5.feet.velocity, 5.feet.acceleration)
//        )
// //                TrajectoryFactory.depotToCargoShipS2
//
//        val controller = RamseteController(2.0, 0.7) // generatedLTVUnicycleController // RamseteController(2.0, 0.7) //defaultLTVUnicycleController
//        val kinematics = DifferentialDriveKinematics(26.inch.inMeters())
//        val odometry = DifferentialDriveOdometry(kinematics)
//
//        odometry.resetPosition(trajectory.states.first().poseMeters)
// //        drive.robotLocation = trajectory.firstState.state.pose // Pose2d(4.feet, 4.feet, 45.degree)
//        var robotPose = Pose2d(0.meters, 0.meters, 0.radians.toRotation2d())
//        var currentTime = 0.second
//        val deltaTime = 20.milli.second
//
//        val xList = arrayListOf<Double>()
//        val yList = arrayListOf<Double>()
//
//        val refXList = arrayListOf<Double>()
//        val refYList = arrayListOf<Double>()
//
//        while (currentTime.inSeconds() < trajectory.totalTimeSeconds) {
//
//            val nextTime = currentTime + deltaTime
//            val state = trajectory.sample(nextTime.inSeconds())
//
//
//            val output = controller.calculate(robotPose, state)
//
//            val newX = robotPose.translation.x
//            val newY = robotPose.translation.y
//
//            xList += newX.meters.inFeet()
//            yList += newY.meters.inFeet()
//
//            val wantedX = state.poseMeters.translation.x.meters
//            val wantedY = state.poseMeters.translation.y.meters
//
//            refXList += wantedX.feet
//            refYList += wantedY.feet
//
//            robotPose = odometry.updateWithTime(currentTime.inSeconds(), robotPose.rotation, kinematics.toWheelSpeeds(output))
//
//            currentTime = nextTime
//
//        }
//
//        val fm = DecimalFormat("#.###").format(trajectory.totalTimeSeconds)
//
//        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
//                .xAxisTitle("X").yAxisTitle("Y").build()
//
//        chart.styler.markerSize = 8
//        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))
//
//        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
//        chart.styler.chartTitlePadding = 15
//
//        chart.styler.xAxisMin = -1.0
//        chart.styler.xAxisMax = refXList.max()
//        chart.styler.yAxisMin = -1.0
//        chart.styler.yAxisMax = refXList.max()
//
//        chart.styler.chartFontColor = Color.WHITE
//        chart.styler.axisTickLabelsColor = Color.WHITE
//
//        chart.styler.legendBackgroundColor = Color.GRAY
//
//        chart.styler.isPlotGridLinesVisible = true
//        chart.styler.isLegendVisible = true
//
//        chart.styler.plotGridLinesColor = Color.GRAY
//        chart.styler.chartBackgroundColor = Color.DARK_GRAY
//        chart.styler.plotBackgroundColor = Color.DARK_GRAY
//
//        chart.addSeries("Trajectory", refXList.toDoubleArray(), refYList.toDoubleArray())
//        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())
//
// //        val terror =
// //            TrajectoryGeneratorTest.trajectory.lastState.state.pose.translation - drive.robotLocation.translation
// //        val rerror = TrajectoryGeneratorTest.trajectory.lastState.state.pose.rotation - drive.robotLocation.rotation
//
// //        System.out.printf("%n[Test] X Error: %3.3f, Y Error: %3.3f%n", terror.x.feet, terror.y.feet)
//
// //        assert(terror.norm.value.also {
// //            println("[Test] Norm of Translational Error: $it")
// //        } < 0.50)
// //        assert(rerror.degree.also {
// //            println("[Test] Rotational Error: $it degrees")
// //        } < 5.0)
//
//        SwingWrapper(chart).displayChart()
//        Thread.sleep(1000000)
//    }
// }
//
