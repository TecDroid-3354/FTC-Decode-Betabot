package org.firstinspires.ftc.teamcode.systems

import Distance
import androidx.core.util.Supplier

data class Point(
    val x: Double,
    val y: Double
)

data class LInterpolationConfig (
    val firstCoordinate: Point,
    val secondCoordinate : Point
)

class LinearInterpolationConstructor(val config: LInterpolationConfig, var distanceToGoal: Supplier<Distance>) {

    fun getDesiredPoint(): Double {
        val y = config.firstCoordinate.y  + ((distanceToGoal.get() as Double - config.firstCoordinate.x).times(
                (config.secondCoordinate.y - config.firstCoordinate.y))).div(
                (config.secondCoordinate.x - config.firstCoordinate.x))

        return y
    }
}