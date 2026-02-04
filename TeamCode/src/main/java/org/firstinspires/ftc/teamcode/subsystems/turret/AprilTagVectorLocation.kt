package org.firstinspires.ftc.teamcode.subsystems.turret

import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Vector2d

object AprilTagVectorLocations {

    // April tag location using FTC Field coordinate system.
    // The origin is located in the exact center of the field.
    // Thus, the blue april tag is located in the 3 Quadrant, both negatives.
    // The red april tag is the same but laying on the positive y axis.

    /**
     * Blue april tag location vector in inches
     */
    val blueGoalCornerVector = Vector2d(72.0, 72.0)

    /**
     * Red april tag location vector in inches
     */
    val redGoalCornerVector = Vector2d(-72.0, 72.0)

    val blueAprilTagLocationVector = Vector2d(-59.138 , 56.67)

    val redAprilTagLocationVector = Vector2d(59.138 , 56.67)
}