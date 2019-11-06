package frc.robot.subsystems.sensors

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.superstructure.Length
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan
import kotlin.properties.Delegates
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian

object LimeLight {

    @Suppress("ControlFlowWithEmptyBody")
    val hasTarget get() = currentState.hasTarget

    val lastSkew get() = currentState.skew

    private val currentStateMutex = Object()

    var currentState = State()
        get() = synchronized(currentStateMutex) { field }
        set(newValue) = synchronized(currentStateMutex) { field = newValue }

    val lastYaw get() = currentState.yaw
    val targetToTheLeft get() = currentState.skew > (-45).degree

    var wantsLEDsOn by Delegates.observable(false) { _, _, wantsOn ->
        ledModeEntry.setDouble(if (wantsOn) 3.0 else 1.0)
    }

    var wantedPipeline by Delegates.observable(0) { _, _, newPipeline ->
        if (newPipeline < 0 || newPipeline > 9) {} else pipelineEntry.setDouble(newPipeline.toDouble())
    }

    var wantsDriverMode by Delegates.observable(false) { _, _, wantsHighExposure ->
        camModeEntry.setDouble(if (wantsHighExposure) 1.0 else 0.0)
    }

    var wantedStreamMode by Delegates.observable(2.0) { _, _, newMode ->
        streamEntry.setDouble(newMode)
    }

    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val tvEntry = table.getEntry("tv")
    private val txEntry = table.getEntry("tx")
    private val tyEntry = table.getEntry("ty")
    private val tsEntry = table.getEntry("ts")
    private val widthEntry = table.getEntry("tlong")
    private val heightEntry = table.getEntry("tshort")
    private val latencyEntry = table.getEntry("tl")
    private val ledModeEntry = table.getEntry("ledMode")
    private val camModeEntry = table.getEntry("camMode")
    private val pipelineEntry = table.getEntry("pipeline")
    private val streamEntry = table.getEntry("stream")

    data class State(
        val hasTarget: Boolean,
        val yaw: SIUnit<Radian>,
        val pitch: SIUnit<Radian>,
        val skew: SIUnit<Radian>,
        val width: Double,
        val height: Double,
        val timestamp: SIUnit<Second>
    ) {
        constructor() : this(false, 0.degree, 0.degree, 0.degree, 0.0, 0.0, 0.second)
    }

    fun configureDisabled() {
        wantsLEDsOn = false
        wantsDriverMode = false
        wantedStreamMode = 2.0
    }

    fun configureEnabled() {
        wantsLEDsOn = true
        wantsDriverMode = false
        wantedStreamMode = 2.0
        wantedPipeline = 1 // TODO add pipelines to the limelight
    }

    init {
        wantedStreamMode = 2.0
    }

    fun tangentDistance(isHatchTarget: Boolean = true): SIUnit<Meter> {
        // Thanks 1678
        val limeHeight = 42.25.inch - 1.inch + 2.inch
        val targetHeight = if (isHatchTarget) 28.6.inch else 36.inch
        val limelightPitch = (-30).degree
        val lastPitch = currentState.pitch
        val distance = tan(
                (lastPitch + limelightPitch).radian *
                        (limeHeight - targetHeight).meter
        ).meter
        return distance
    }

    private fun focalLenDistance(isHighRes: Boolean = true): Length {
        val focalLen = 707.0 * (57.0 / 53.0) // = (isHighRes) ? x_focal_length_high : x_focal_length_low;
        val width = 14.6.inch
        val targetSizePx = LimeLight.currentState.width // table.getEntry("tlong").getDouble(0.0) // getTargetXPixels();
        val hypotenuse = width * focalLen / targetSizePx * (/*720p vs 240p*/ if (!isHighRes) 240.0 / 720.0 else 1.0)
        val deltaElevation = (45 - 29).inch
        // since a^2 + b^2 = c^2, we find a^2 = c^2 - b^2
        return sqrt(
                hypotenuse.meter.pow(2) - deltaElevation.meter.pow(2)
        ).meter
    }

    fun estimateDistance() = focalLenDistance(isHighRes())

    private fun isHighRes(): Boolean {
        return wantedPipeline == 0
    }

    fun update() {

        val newState = State(
                tvEntry.getDouble(0.0) == 1.0,
                -txEntry.getDouble(0.0).degree,
                tyEntry.getDouble(0.0).degree,
                tsEntry.getDouble(0.0).degree,
                widthEntry.getDouble(0.0),
                heightEntry.getDouble(0.0),
                Timer.getFPGATimestamp().second - latencyEntry.getDouble(0.0).milli.second - 11.milli.second
        )
        this.currentState = newState

//        val distance = estimateDistance(true)
//        val tx = newState.yaw
//        val rawPose = Pose2d(Translation2d(distance, tx.toRotation2d()))
//
//        try { val targetPose = if (!(rawPose.translation.x.absoluteValue > (Constants.kRobotLength / 2.0 - 5.inch) ||
//                        rawPose.translation.y.absoluteValue > (Constants.kRobotWidth / 2.0))) null else
//            DriveSubsystem.localization[newState.timestamp] + (Constants.kCenterToFrontCamera + rawPose)
//
//            TargetTracker.addSamples(
//                    newState.timestamp.second, listOfNotNull(targetPose)
//            )
//        } catch (e: Exception) {
//            println("[LimeLight] Could not add new target! ${e.localizedMessage}")
//            e.printStackTrace()
//        }
    }
}
