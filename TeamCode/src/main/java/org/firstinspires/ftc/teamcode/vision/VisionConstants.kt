package org.firstinspires.ftc.teamcode.vision

import Angle
import Distance

class VisionConstants {
    object LimelightIdentification {
        const val Id: String = "limelight"
    }

    object LimelightConfiguration {
        // Rate at which the limelight data is retrieved. Must range from 1 - 250.
        val PollRateHz: Int = 250
        val PipelineIndex: Int = 0
    }

    // Get these values from design
    object LimelightPhysicalDescription {
        val LLMountAngleFromHorizontal: Angle = Angle.fromDegrees(0.0)
        val LLHeightFromGroundToLens: Distance = Distance.fromInches(0.0)
    }

    // Retrieved from Decode Manual
    // https://ftc-resources.firstinspires.org/ftc/game/manual
    object AprilTagsIdentification {
        val ObeliskIds: List<Int> = listOf(21, 22, 23)
        val BlueGoalId: Int = 20
        val RedGoalId: Int = 24
    }

    // Retrieved from Decode Manual
    // https://ftc-resources.firstinspires.org/ftc/game/manual
    object AprilTagsPhysicalDescription {
        // Distance to the top of the Obelisk is 23 inches, but we need the center of the aprilTag,
        // which is 8.125 inches tall
        val ObeliskHeightFromGround: Distance = Distance.fromInches(23.0 - (8.125)/2.0)

        // Distance to the top of the rectangle part of the GOAL is 38.75 inches, but we need the
        // center of the aprilTag, which is 9.25 inches below
        val BlueGoalHeightFromGround: Distance = Distance.fromInches(38.75 - 9.25)

        // Distance to the top of the rectangle part of the GOAL is 38.75 inches, but we need the
        // center of the aprilTag, which is 9.25 inches below
        val RedGoalHeightFromGround: Distance = Distance.fromInches(38.75 - 9.25)
    }

    /* Contains any tolerance and setpoint necessary for alignment with vision */
    object AlignmentParameters {
        val ShootingHorizontalTolerance: Distance = Distance.fromInches(0.0)
    }
}