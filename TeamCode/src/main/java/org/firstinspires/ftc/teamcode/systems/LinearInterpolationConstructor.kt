package org.firstinspires.ftc.teamcode.systems

import Angle
import Distance
import androidx.core.util.Supplier

data class Pointer(
    val xDistanceToGoal: Distance,
    val yHoodAngle: Angle
)

data class LInterpolationConfig (
    val firstCoordinate: Pointer,
    val secondCoordinate: Pointer
)

class LinearInterpolationConstructor(val config: LInterpolationConfig, var distanceToGoal: Supplier<Distance>) {

    fun getDesiredPoint(): Double {
        val y = config.firstCoordinate.yHoodAngle.rotations +
                ((distanceToGoal.get().meters - config.firstCoordinate.xDistanceToGoal.meters)
                    .times(
                        (config.secondCoordinate.yHoodAngle.rotations - config.firstCoordinate.yHoodAngle.rotations))).div(
                        (config.secondCoordinate.xDistanceToGoal.meters - config.firstCoordinate.xDistanceToGoal.meters))

        return y
    }
}