@file:Suppress("UNUSED_VARIABLE")

package misc

fun main() {

    val scoutersPerAlliance = 3
    val rotations = 11
    val breaks = 0

    val totalScouters = arrayListOf(
            "A", "B", "C", "D", "E", "F", "G", "H", "I"
    )

    val currentlyScouting = ArrayList(totalScouters.subList(0, scoutersPerAlliance * 2))
    val onBreak = ArrayList(totalScouters.subList(scoutersPerAlliance * 2, totalScouters.size))

    val chonkSchedule = mutableListOf<List<String>>()

    println("currently scouting $currentlyScouting, on break $onBreak")

    var lastBreakIndex = 0

    var currentRotation = 0

    while (currentRotation <rotations) {
        // loop over  every rotation of scouters starting with 1
        // we'll add them to the list then sift them over
//        chonkSchedule += currentlyScouting

        // the person at the top of the list os coming off break, so let's store them and pop them off
        val humanComingOffBreak = onBreak.first()
        onBreak.remove(onBreak.first())

        // the person going on break is the person at the index
        val humanToGoOnBreak = currentlyScouting[lastBreakIndex]

        // replace the currently scouting human with the guy who got popped off the list
        currentlyScouting[lastBreakIndex] = humanComingOffBreak

        // and add him to the bottom of the list.
        onBreak.add(humanToGoOnBreak)

//        println("$humanComingOffBreak is coming off break and $humanToGoOnBreak is going on break")

        lastBreakIndex += 1
        if (lastBreakIndex >= currentlyScouting.size) lastBreakIndex = 0

        currentRotation += 1

        println("currently scouting $currentlyScouting, on break $onBreak")
    }

    println("chonk schedule $chonkSchedule")
}
