package org.firstinspires.ftc.teamcode.systems.shooterSystem

import Angle
import AngularVelocity
import Distance
import com.seattlesolvers.solverslib.command.SubsystemBase
import java.util.function.Supplier
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingDouble
import org.firstinspires.ftc.teamcode.utils.interpolation.InterpolatingTreeMap

// The following data class allows us to declare points that take a distance and return an angle
data class HoodPoint(
    val xDistanceToGoal: Distance,
    val yHoodAngle: Angle
)

data class ShooterPoint(
    val xDistanceToGoal: Distance,
    val yShooterRPM: AngularVelocity
)

class InterpolationConstructor(private var distanceToGoal: Supplier<Distance>, subsystem: String) {
    // This @maximumSize param represents the max number of allowed elements in the tree map
    // Once this number is surpassed, old data, i.e. the oldest data entries, is removed
    // to allow new entries to come in
    private var map: InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> = InterpolatingTreeMap(10)

    // The following points variable is a list containing all the points we want to interpolate between
    private val hoodPoints = listOf(
        // todo: !!!!! SET ALL DEGREES !!!!!!!!!
        HoodPoint(Distance.fromInches(0.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(5.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(10.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(15.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(20.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(25.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(30.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(35.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(40.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(45.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(50.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(55.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(60.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(65.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(70.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(75.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(80.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(85.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(90.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(95.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(100.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(105.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(110.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(115.0), Angle.fromDegrees(0.0)),
        HoodPoint(Distance.fromInches(120.0), Angle.fromDegrees(0.0)),
    )

    private val shooterPoints = listOf(
        //TODO: GET RPMs
        ShooterPoint(Distance.fromInches(0.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(5.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(10.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(15.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(20.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(25.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(30.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(35.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(40.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(45.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(50.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(55.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(60.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(65.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(70.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(75.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(80.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(85.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(90.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(95.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(100.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(105.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(110.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(115.0), AngularVelocity.fromRpm(0.0)),
        ShooterPoint(Distance.fromInches(120.0), AngularVelocity.fromRpm(0.0)),
    )

    init {
        // todo: test
        when(subsystem) {
            "hood" -> {
                // Adding data to the tree map. It is necessary to use the InterpolatingDouble() since
                // it gives extra functionality to doubles by allowing them to actually interpolate
                for (point in hoodPoints) {
                    map.put(
                        InterpolatingDouble(point.xDistanceToGoal.inches),
                        InterpolatingDouble(point.yHoodAngle.degrees))
                }
            }

            "shooter" -> {
                // Adding data to the tree map. It is necessary to use the InterpolatingDouble() since
                // it gives extra functionality to doubles by allowing them to actually interpolate
                for (point in shooterPoints) {
                    map.put(
                        InterpolatingDouble(point.xDistanceToGoal.inches),
                        InterpolatingDouble(point.yShooterRPM.rpm))
                }
            }
        }
    }

    fun getDesiredPoint(): Double {
        return map.getInterpolated(InterpolatingDouble(distanceToGoal.get().inches)).value
    }
}