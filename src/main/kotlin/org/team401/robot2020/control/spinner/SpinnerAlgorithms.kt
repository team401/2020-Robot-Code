package org.team401.robot2020.control.spinner

import com.revrobotics.ColorMatchResult
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.MeasureUnitless
import org.snakeskin.measure.Unitless
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.team401.robot2020.subsystems.SpinnerSubsystem.SpinnerColor
import org.team401.robot2020.config.*
import org.team401.robot2020.config.constants.SpinnerConstants

object SpinnerAlgorithms {
    private val degreesPerSlice = 45.0.Degrees

    fun calculateSpinnerRotation(currentColor: SpinnerColor, desiredColor: SpinnerColor): AngularDistanceMeasureDegrees {

        val currentPosition = findPosition(currentColor) * degreesPerSlice
        val desiredPosition = findPosition(desiredColor) * degreesPerSlice
        val difference = desiredPosition - currentPosition - 90.0.Degrees

        if (difference <= (-90.0).Degrees) {
            return difference + 180.0.Degrees
        }

        return difference

    }

    private fun findPosition(color: SpinnerColor): MeasureUnitless {

        return when (color) {
            SpinnerColor.Red -> 0.0.Unitless
            SpinnerColor.Green -> 1.0.Unitless
            SpinnerColor.Blue -> 2.0.Unitless
            SpinnerColor.Yellow -> 3.0.Unitless
            else -> (-1.0).Unitless
        }
    }

    fun matchColor(matchResult: ColorMatchResult): SpinnerColor {

        return when(matchResult.color) {
            SpinnerConstants.blueColor -> SpinnerColor.Blue
            SpinnerConstants.redColor -> SpinnerColor.Red
            SpinnerConstants.greenColor -> SpinnerColor.Green
            SpinnerConstants.yellowColor -> SpinnerColor.Yellow
            else -> SpinnerColor.Unknown
        }
    }
}