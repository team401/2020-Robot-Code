 package org.team401.robot2020.control.spinner

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees

object SpinnerAlgorithm {

    private val degreesPerSlice = 45.0.Degrees

    fun calculateSpinnerRotation(currentColor: SpinnerColor, desiredColor: SpinnerColor): AngularDistanceMeasureDegrees {

        val currentPosition = findPosition(currentColor)
        val desiredPosition = findPosition(desiredColor)
        var difference = currentPosition - desiredPosition

        if (difference > 2) {
            difference -= 4
        } else if (difference < -2) {
            difference += 4
        }

        return (difference * degreesPerSlice.value).Degrees

    }

    private fun findPosition(color: SpinnerColor): Int {

        return when (color) {
            SpinnerColor.Red -> 1
            SpinnerColor.Green -> 2
            SpinnerColor.Blue -> 3
            SpinnerColor.Yellow -> 4
        }

    }

}

 fun main() {
     println(SpinnerAlgorithm.calculateSpinnerRotation(SpinnerColor.Red, SpinnerColor.Red))
 }