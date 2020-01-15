package org.team401.robot2020.control.spinner

import com.revrobotics.ColorMatchResult
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.team401.robot2020.subsystems.SpinnerSubsystem.SpinnerColor
import org.team401.robot2020.config.*

object SpinnerAlgorithms {
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
            else -> -1
        }
    }

    fun matchColor(matchResult: ColorMatchResult): SpinnerColor {

        return when(matchResult.color) {
            SpinnerParameters.blueTarget -> SpinnerColor.Blue
            SpinnerParameters.redTarget -> SpinnerColor.Red
            SpinnerParameters.greenTarget -> SpinnerColor.Green
            SpinnerParameters.yellowTarget -> SpinnerColor.Yellow
            else -> SpinnerColor.Unknown
        }
    }
}