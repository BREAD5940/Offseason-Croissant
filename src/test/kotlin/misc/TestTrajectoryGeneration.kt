package misc

import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.auto.paths.TrajectoryFactory
import org.junit.Test

class TestTrajectoryGeneration {

//    @Test fun testPrint() {
//
//        val w = listOf(
//                Pose2d(22.86745.feet, 3.444882.feet, (-151.25).degrees).asWaypoint(),
//                Pose2d(24.467.feet, 3.018.feet, (160).degrees).asWaypoint()
//        ).map { it.position }
//        val config = FalconTrajectoryConfig(6.feet.velocity, 6.feet.acceleration)
//                .addConstraints(DifferentialDriveVoltageConstraint(
//                        Constants.DriveConstants.kLeftTransmissionModelHighGear.toSimpleFeedforward(),
//                        Constants.DriveConstants.kinematics,
//                        TrajectoryFactory.kMaxVoltage.value,
//                        Constants.DriveConstants.kTrackWidth.inMeters()))
//                .setReversed(true)
//                .setKinematics(Constants.DriveConstants.kinematics)
//
//        val traject = TrajectoryGenerator.generateTrajectory(w, config)
//    }

    @Test fun testTrajectoryGeneration() {

        var t: Trajectory
        var start = System.currentTimeMillis() / 1000.0

        t = TrajectoryFactory.cargoShipFLToRightLoadingStation
        t = TrajectoryFactory.cargoShipFLToLeftLoadingStation
        t = TrajectoryFactory.cargoShipFRToRightLoadingStation
        t = TrajectoryFactory.cargoShipS1ToDepot
        t = TrajectoryFactory.cargoShipS1ToLoadingStation
        t = TrajectoryFactory.centerStartToCargoShipFL
        t = TrajectoryFactory.depotToCargoShipS2
        t = TrajectoryFactory.loadingStationToCargoShipFR
        t = TrajectoryFactory.loadingStationToCargoShipS2
        t = TrajectoryFactory.loadingStationToRocketFPrep
        t = TrajectoryFactory.loadingStationToRocketN
        t = TrajectoryFactory.loadingStationReversedToRocketNPrep
        t = TrajectoryFactory.rocketNPrepToRocketN
        t = TrajectoryFactory.rocketNToDepot
        t = TrajectoryFactory.rocketFPrepareToRocketF
        t = TrajectoryFactory.rocketFToRocketFPrepare // TODO fixme
        t = TrajectoryFactory.rocketFPrepareToLoadingStation
        t = TrajectoryFactory.rocketFToDepot // TODO fixme
        t = TrajectoryFactory.rocketFToLoadingStation // TODO fixme
        t = TrajectoryFactory.rocketNToLoadingStation
        t = TrajectoryFactory.sideStartToCargoShipS1Prep
        t = TrajectoryFactory.cargoS1PrepToCargoS1
        t = TrajectoryFactory.cargoShipS1ToS1Prep // TODO fixme
        t = TrajectoryFactory.cargoS1PrepToLoadingStation
        t = TrajectoryFactory.loadingStationToCargoS2Prep
        t = TrajectoryFactory.cargoPrepToCargoS2
        t = TrajectoryFactory.sideStartToRocketF
        t = TrajectoryFactory.sideStartReversedToRocketFPrepare
        t = TrajectoryFactory.testTrajectory

        var delta = (System.currentTimeMillis() / 1000.0) - start
        println("Generated in $delta")
    }

}