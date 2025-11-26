package org.firstinspires.ftc.teamcode.subsystems.indexer

import AngularVelocity
import LinearVelocity
import kotlin.math.PI

class IndexerConstants {
    object Ids {
        const val servoFront: String = "frontRightMotor"
        const val servoRight: String = "frontLeftMotor"
        const val servoLeft: String = "backRightMotor"

        const val colorSensorFront: String = "frontRightMotor"
        const val colorSensorRight: String = "frontLeftMotor"
        const val colorSensorLeft: String = "backRightMotor"

    }

    object Positions {
        const val feedPosition: Double = 1.0
        const val homePosition: Double = 0.0
    }
}