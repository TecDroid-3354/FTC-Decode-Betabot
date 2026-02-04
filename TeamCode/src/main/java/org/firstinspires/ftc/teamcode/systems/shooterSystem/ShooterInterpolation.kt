package org.firstinspires.ftc.teamcode.systems.shooterSystem

import Angle
import Distance
import java.util.function.Supplier
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingDouble
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingTreeMap

// The following data class allows us to declare points that take a distance and return an angle
data class Point(
    val xDistanceToGoal: Distance,
    val yHoodAngle: Angle
)

class ShooterInterpolationConstructor(private var distanceToGoal: Supplier<Distance>) {
    // This @maximumSize param represents the max number of allowed elements in the tree map
    // Once this number is surpassed, old data, i.e. the oldest data entries, is removed
    // to allow new entries to come in
    private var map: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap(10)

    // The following points variable is a list containing all the points we want to interpolate between
    private val points = listOf(
        // todo: finish adding all points
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        Point(Distance.fromInches(0.0), Angle.fromDegrees(0.0))
    )

    init {
        // Adding data to the tree map. It is necessary to use the InterpolatingDouble() since
        // it gives extra functionality to doubles by allowing them to actually interpolate
        for (point in points) {
            map.put(
                InterpolatingDouble(point.xDistanceToGoal.inches),
                InterpolatingDouble(point.yHoodAngle.degrees))
        }
    }

    fun getDesiredPoint(): Double {
        return map.getInterpolated(InterpolatingDouble(distanceToGoal.get().inches)).value
    }
}