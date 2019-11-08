package org.team5940.pantry.lib

import com.github.salomonbrys.kotson.fromJson
import com.github.salomonbrys.kotson.get
import com.google.gson.Gson
import com.google.gson.JsonObject
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derived.acceleration
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.second

@Suppress("SpellCheckingInspection")
val kGson = Gson()

fun TimedTrajectory<Pose2dWithCurvature>.toJson() = kGson.toJson(this@toJson)

fun jsonToTrajectory(trajectory: String): TimedTrajectory<Pose2dWithCurvature> {

    val jsonData = kGson.fromJson<JsonObject>(trajectory)
    val reversed = jsonData["reversed"].asBoolean
    val points = jsonData["points"].asJsonArray
    val trajectoryList = arrayListOf<TimedEntry<Pose2dWithCurvature>>()

    points.forEach { sample ->
        val state = sample["state"]
        val pose = state["pose"]
        val translation = pose["translation"]
        val x = translation["x"].asDouble.meter
        val y = translation["y"].asDouble.meter
        val rotation = pose["rotation"]["value"].asDouble
        val curvature = state["curvature"].asDouble
        val dkds = state["dkds"].asDouble
        val time = sample["_t"].asDouble
        val velocity = sample["_velocity"].asDouble
        val acceleration = sample["_acceleration"].asDouble

        val pose2d = Pose2dWithCurvature(
                Pose2d(Translation2d(x, y), Rotation2d(rotation)),
                curvature, dkds)
        val entry = TimedEntry(
                pose2d,
                time.second,
                velocity.meter.velocity,
                acceleration.meter.acceleration)
        trajectoryList.add(entry)
    }

    return TimedTrajectory(
            trajectoryList,
            reversed)
}
