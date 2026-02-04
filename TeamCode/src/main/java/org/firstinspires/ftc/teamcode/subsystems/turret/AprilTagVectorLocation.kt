package org.firstinspires.ftc.teamcode.subsystems.turret

import com.seattlesolvers.solverslib.geometry.Vector2d

object AprilTagVectorLocations {

    // April tag location using FTC Field coordinate system.
    // The origin is located in the exact center of the field.
    // Thus, the blue april tag is located in the 3 Quadrant, both negatives.
    // The red april tag is the same but laying on the positive y axis.

    /**
     * Blue april tag location vector in inches
     */
    val blueAprilTag = Vector2d(72.0, 72.0)

    /**
     * Red april tag location vector in inches
     */
    val redAprilTag = Vector2d(-72.0, 72.0)
}