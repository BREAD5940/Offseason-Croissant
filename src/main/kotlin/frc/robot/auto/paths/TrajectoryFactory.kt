package frc.robot.auto.paths

import frc.robot.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch

object TrajectoryFactory {

    /** Constraints **/

    val kMaxVelocity = 7.5.feet.velocity
    val kMaxAcceleration = 6.feet.acceleration

    private val kMaxHabitatVelocity = 3.5.feet.velocity

    private val kFirstPathMaxAcceleration = 6.feet.acceleration

    private val kVelocityRadiusConstraintRadius = 4.5.feet
    private val kVelocityRadiusConstraintVelocity = 3.75.feet.velocity

    private val kMaxCentripetalAccelerationElevatorUp = 6.feet.acceleration
    private val kMaxCentripetalAccelerationElevatorDown = 7.5.feet.acceleration

    val kMaxVoltage = 10.volt

    /** Adjusted Poses **/

    private val cargoShipFLAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kCargoShipFL,
            transform = Constants.kForwardIntakeToCenter
    )
    private val cargoShipFRAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kCargoShipFR,
            transform = Constants.kForwardIntakeToCenter,
            translationalOffset = Translation2d(0.inch, 5.inch)
    )
    private val cargoShipS1Adjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kCargoShipS1,
            transform = Constants.kForwardIntakeStowedToCenter,
            translationalOffset = Translation2d(1.9.inch, 0.inch)
    )
    private val cargoShipS2Adjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kCargoShipS2,
            transform = Constants.kForwardIntakeToCenter,
            translationalOffset = Translation2d(1.9.inch, 1.5.inch)
    )
    private val cargoShipS3Adjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kCargoShipS3,
            transform = Constants.kForwardIntakeToCenter
    )
    private val depotAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kDepotBRCorner,
            transform = Constants.kBackwardIntakeToCenter
    )
    private val loadingStationAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kLoadingStation,
            transform = Constants.kBackwardIntakeToCenter,
            translationalOffset = Translation2d((-9).inch, 0.inch)
    )
    private val loadingStationUnPassedthroughAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kLoadingStationReversed,
            transform = Constants.kForwardIntakeStowedToCenter,
            translationalOffset = Translation2d((-18.5).inch, 0.inch)
    )
    private val rocketFAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kRocketF,
            transform = Constants.kForwardIntakeToCenter.transformBy(Pose2d(-8.inch, -3.inch)),
            translationalOffset = Translation2d(-5.inch, -1.inch)
    )
    val rocketNAdjusted = TrajectoryWaypoints.Waypoint(
            trueLocation = TrajectoryWaypoints.kRocketN,
            transform = Constants.kForwardIntakeToCenter.transformBy(Pose2d(13.inch, 0.inch))
    )

    /** Trajectories **/

    val cargoShipFLToRightLoadingStation by lazy {
        generateTrajectory(
                true,
                listOf(
                        cargoShipFLAdjusted,
                        cargoShipFLAdjusted.position.transformBy(Pose2d((-0.7).feet, 0.feet)).asWaypoint(),
                        Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
                        loadingStationAdjusted
                ),
                getConstraints(false, loadingStationAdjusted), 8.feet.velocity, 6.feet.acceleration, kMaxVoltage
        )
    }

    val cargoShipFLToLeftLoadingStation by lazy { generateTrajectory(
            true,
            listOf(
                    cargoShipFLAdjusted,
                    cargoShipFLAdjusted.position.transformBy(Pose2d((-0.7).feet, 0.feet)).asWaypoint(),
                    Pose2d(10.6.feet, 6.614.feet, 69.degree).mirror.asWaypoint(),
                    loadingStationAdjusted.position.mirror.asWaypoint()
            ),
            getConstraints(false, loadingStationAdjusted), 8.feet.velocity, 6.feet.acceleration, kMaxVoltage
    ) }

    val cargoShipFRToRightLoadingStation = cargoShipFLToLeftLoadingStation.mirror()

    val cargoShipS1ToDepot by lazy { generateTrajectory(
            true,
            listOf(
                    cargoShipS1Adjusted,
                    Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
                    depotAdjusted
            ),
            getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val cargoShipS1ToLoadingStation by lazy { generateTrajectory(
            true,
            listOf(
                    cargoShipS1Adjusted,
                    Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
                    loadingStationAdjusted
            ),
            getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val centerStartToCargoShipFL by lazy { generateTrajectory(
            false,
            listOf(
                    TrajectoryWaypoints.kCenterStart.asWaypoint(),
                    cargoShipFLAdjusted
            ),
            getConstraints(false, cargoShipFLAdjusted), kMaxVelocity, 4.feet.acceleration, kMaxVoltage
    ) }

//    val centerStartToCargoShipFR = centerStartToCargoShipFL.mirror()

    val depotToCargoShipS2 by lazy { generateTrajectory(
            false,
            listOf(
                    depotAdjusted,
                    Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
                    cargoShipS2Adjusted
            ),
            getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val loadingStationToCargoShipFR by lazy { generateTrajectory(
            false,
            listOf(
                    loadingStationAdjusted,
                    Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
                    cargoShipFRAdjusted.position.transformBy(Pose2d((-30).inch, 0.inch)).asWaypoint(),
                    cargoShipFRAdjusted
            ),
            getConstraints(false, cargoShipFRAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val loadingStationToCargoShipS2 by lazy { generateTrajectory(
            false,
            listOf(
                    loadingStationAdjusted,
                    Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
                    cargoShipS2Adjusted
            ),
            getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val loadingStationToRocketF by lazy { generateTrajectory(
            false,
            listOf(
                    loadingStationAdjusted,
                    Pose2d(17.039.feet, 6.378.feet, 9.degree).asWaypoint(),
                    rocketFAdjusted
            ),
            getConstraints(true, rocketFAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val loadingStationToRocketN by lazy { generateTrajectory(
            true,
            listOf(
                    loadingStationAdjusted,
                    rocketNAdjusted
            ),
            getConstraints(true, rocketNAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val loadingStationReversedToRocketNPrep by lazy { generateTrajectory(
            true,
            listOf(
                    loadingStationUnPassedthroughAdjusted,
//                    Pose2d(7.55.feet, 2.63.feet, -172.428.degree).asWaypoint()
                    rocketNPrep
            ),
            getConstraints(true, rocketNAdjusted), kMaxVelocity, kMaxAcceleration * 2.0, kMaxVoltage,
            endVelocity = 0.feet.velocity
    ) }

    val rocketNPrepTranslation = Translation2d(13.417.feet, 3.354.feet)

    val rocketNPrep = Pose2d(rocketNPrepTranslation, (-157.435).degree)
            .transformBy(Pose2d(0.inch, 0.inch, 0.degree)).asWaypoint()

    val rocketNPrepRotated = Pose2d(rocketNPrepTranslation, (-30).degree).asWaypoint()

    val rocketNPrepToRocketN by lazy { generateTrajectory(
            false,
            listOf(
//                    rocketNPrep, (-28.75)
                    rocketNPrepRotated,
                    rocketNAdjusted
            ),
            getConstraints(true, rocketNAdjusted, 3.feet.velocity, 4.5.feet), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val rocketNToDepot by lazy { generateTrajectory(
            true,
            listOf(
                    rocketNAdjusted,
                    depotAdjusted
            ),
            getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val rocketFPrepareToRocketF by lazy { generateTrajectory(
            false,
            listOf(
                    rocketFPrepare,
//                    rocketFAdjusted
                    Pose2d(22.312.feet, 2.82.feet, (-151.25).degree).transformBy(Pose2d(9.inch, 4.inch, 0.degree)).asWaypoint()
            ),
            getConstraints(false, Pose2d()), 1.5.feet.velocity, kMaxAcceleration / 1.5, kMaxVoltage
    ) }

    val rocketFToRocketFPrepare by lazy { generateTrajectory(
            reversed = true,
            points = listOf(
                    rocketFAdjusted,
                    Pose2d(24.467.feet, 3.018.feet, (160).degree).asWaypoint()
            ),
            constraints = getConstraints(false, Pose2d()),
            maxVelocity = 3.feet.velocity,
            maxAcceleration = kMaxAcceleration,
            maxVoltage = kMaxVoltage
    ) }

    val rocketFPrepareToLoadingStation by lazy { generateTrajectory(
            false,
            listOf(
                    Pose2d(24.467.feet, 3.018.feet, (160).degree).asWaypoint(),
                    Pose2d(19.216.feet, 5.345.feet, 185.degree).asWaypoint(),
                    Pose2d(8.318.feet, 3.157.feet, 180.degree).asWaypoint(),
                    loadingStationUnPassedthroughAdjusted
            ),
            getConstraints(false, loadingStationUnPassedthroughAdjusted.position,
                    3.feet.velocity, 6.feet), 10.feet.velocity,
            kMaxAcceleration * 1.75, 10.5.volt
    ) }

    val rocketFToDepot by lazy { generateTrajectory(
            true,
            listOf(
                    rocketFAdjusted,
                    Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
                    depotAdjusted
            ),
            getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val rocketFToLoadingStation by lazy { generateTrajectory(
            true,
            listOf(
                    rocketFAdjusted,
                    Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
                    loadingStationAdjusted
            ),
            getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val rocketNToLoadingStation by lazy { generateTrajectory(
            true,
            listOf(
                    rocketNAdjusted,
                    loadingStationAdjusted
            ),
            getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    ) }

    val sideStartToCargoShipS1 by lazy { generateTrajectory(
            false,
            listOf(
                    TrajectoryWaypoints.kSideStart.asWaypoint(),
                    TrajectoryWaypoints.kSideStart.transformBy(Pose2d(4.feet, 0.feet, 0.degree)).asWaypoint(),
                    Pose2d(18.123.feet, 5.691.feet, (-15.03).degree).asWaypoint(),
                    cargoShipS1Adjusted.position.transformBy(Pose2d((-1).feet, 0.feet, 0.degree)).asWaypoint(),
                    cargoShipS1Adjusted.position.transformBy(Pose2d((6).inch, 0.feet, 0.degree)).asWaypoint()
            ),
            getConstraints(true, cargoShipS1Adjusted), kMaxVelocity, kMaxAcceleration * 1.5, kMaxVoltage
    ) }

    val s1PrepTranslation = Pose2d(23.992.feet, 7.121.feet, (-168.246).degree).asWaypoint()

    val cargoShipS1ToS1Prep by lazy { generateTrajectory(
            true,
            listOf(
                    cargoShipS1Adjusted,
                    Pose2d(15.831.feet, 5.489.feet, (-165.226).degree).asWaypoint(),
                    s1PrepTranslation
//                    cargoShipS1Adjusted.position.transformBy(Pose2d((-1).feet, 0.feet, 0.degree)).asWaypoint(),
            ),
            getConstraints(true, Pose2d()), 9.feet.velocity, kMaxAcceleration * 2.0, kMaxVoltage
    ) }

    val cargoS1PrepToLoadingStation by lazy { generateTrajectory(
            false,
            listOf(
                    s1PrepTranslation,
                    loadingStationUnPassedthroughAdjusted
//                    cargoShipS1Adjusted.position.transformBy(Pose2d((-1).feet, 0.feet, 0.degree)).asWaypoint(),
            ),
            getConstraints(true, loadingStationUnPassedthroughAdjusted), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    ) }

    val offsetCargoS1 = cargoShipS1Adjusted.position.transformBy(Pose2d(-2.feet, 0.feet, 0.degree)).translation

    val loadingStationToCargoS1Prep by lazy { generateTrajectory(
            true,
            listOf(
                    loadingStationUnPassedthroughAdjusted,
                    Pose2d(offsetCargoS1, -180.degree).asWaypoint()
            ),
            getConstraints(true, Pose2d()), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    ) }

    val cargoPrepToCargoS1 by lazy { generateTrajectory(
            false,
            listOf(
                    Pose2d(offsetCargoS1, 90.degree).asWaypoint(),
                    cargoShipS1Adjusted
            ),
            getConstraints(true, loadingStationUnPassedthroughAdjusted), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    ) }

    val sideStartToRocketF by lazy { generateTrajectory(
            reversed = false,
            points = listOf(
                    Pose2d(TrajectoryWaypoints.kSideStart.translation).asWaypoint(),
                    rocketFAdjusted
            ),
            constraints = getConstraints(false, rocketFAdjusted),
            maxVelocity = kMaxVelocity,
            maxAcceleration = kMaxAcceleration,
            maxVoltage = kMaxVoltage
    ) }

    val rocketFPrepTransform = Pose2d(23.809.feet, 3.399.feet, -143.degree).transformBy(Pose2d((-3).inch, -3.inch, 0.degree)).translation

    val rocketFPrepare = TrajectoryWaypoints.Waypoint(
            Pose2d(rocketFPrepTransform, (-143).degree),
            transform = Pose2d((-3).inch, (-12).inch, 0.degree) // x is forward, y is left (so towards rocket)
    )

    val rocketFPrepareRotated = Pose2d(rocketFPrepTransform, 127.862.degree)//.transformBy(Pose2d((-3).inch, -3.inch, 0.degree))

    val sideStartReversedToRocketFPrepare by lazy { generateTrajectory(
            true,
            listOf(
                    TrajectoryWaypoints.kSideStartReversed.asWaypoint(),
                    Pose2d(15.214.feet, 8.7.feet, 165.degree).asWaypoint(),
//                    Pose2d(20.82.feet, 4.849.feet, 145.651.degree).asWaypoint(),
                    rocketFPrepareRotated.transformBy(Pose2d((-0).inch, 0.inch, 0.degree)).asWaypoint()
            ),
            getConstraints(false, Pose2d()), 9.feet.velocity, 7.feet.acceleration * 1.5, 9.volt
    ) }

    val testTrajectory by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(1.5.feet, 23.feet, 0.degree).asWaypoint(),
                        Pose2d(11.5.feet, 23.feet, 0.degree).asWaypoint()
                ),
                getConstraints(false, Pose2d(100.feet, 100.feet, 0.degree)), kMaxVelocity, 7.feet.acceleration, kMaxVoltage
        )
    }

    /** Generation **/

    private fun getConstraints(
        elevatorUp: Boolean,
        trajectoryEndpoint: Pose2d,
        velocityRadiusConstraintVelocity: SIUnit<Velocity<Meter>> = kVelocityRadiusConstraintVelocity,
        velocityRadius: SIUnit<Meter> = kVelocityRadiusConstraintRadius
    ) =
            listOf(
                    CentripetalAccelerationConstraint(
                            if (elevatorUp)
                                kMaxCentripetalAccelerationElevatorUp
                            else
                                kMaxCentripetalAccelerationElevatorDown
                    ),
                    VelocityLimitRadiusConstraint(
                            trajectoryEndpoint.translation,
                            velocityRadius,
                            velocityRadiusConstraintVelocity
                    ),
                    VelocityLimitRegionConstraint(TrajectoryWaypoints.kHabitatL1Platform, kMaxHabitatVelocity)
            )

    fun getConstraints(
        elevatorUp: Boolean,
        trajectoryEndpoint: TrajectoryWaypoints.Waypoint,
        velocityRadiusConstraintVelocity: SIUnit<Velocity<Meter>> = kVelocityRadiusConstraintVelocity,
        velocityRadius: SIUnit<Meter> = kVelocityRadiusConstraintRadius
    ) =
            getConstraints(elevatorUp, trajectoryEndpoint.position, velocityRadiusConstraintVelocity, velocityRadius)

    fun generateTrajectory(
        reversed: Boolean,
        points: List<TrajectoryWaypoints.Waypoint>,
        constraints: List<TimingConstraint<Pose2dWithCurvature>>,
        maxVelocity: SIUnit<Velocity<Meter>>,
        maxAcceleration: SIUnit<Acceleration<Meter>>,
        maxVoltage: SIUnit<Volt>,
        optimizeCurvature: Boolean = true,
        endVelocity: SIUnit<Velocity<Meter>> = 0.inch.velocity
    ): TimedTrajectory<Pose2dWithCurvature> {

        val driveDynamicsConstraint = DifferentialDriveDynamicsConstraint(Constants.DriveConstants.kHighGearDifferentialDrive, maxVoltage)
        val allConstraints = ArrayList<TimingConstraint<Pose2dWithCurvature>>()

        allConstraints.add(driveDynamicsConstraint)
        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)

        return DefaultTrajectoryGenerator.generateTrajectory(
                points.map { it.position },
                allConstraints,
                0.inch.velocity,
                endVelocity,
                maxVelocity,
                maxAcceleration,
                reversed,
                optimizeCurvature
        )
    }
}

fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
