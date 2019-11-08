package org.team5940.pantry.lib

object SIRotationConstants {
    const val kDegreesToRadians = 0.017453292519943295
    const val kRadianToDegrees = 57.29577951308232
}

val Number.degreeToRadian get() = toDouble() * SIRotationConstants.kDegreesToRadians
val Number.radianToDegree get() = toDouble() * SIRotationConstants.kRadianToDegrees
