package org.firstinspires.ftc.teamcode.subsystems.indexer

class IndexerConstants {
    object Ids {
        const val frontServo: String = "frontServo"
        const val rightServo: String = "rightServo"
        const val leftServo: String = "leftServo"

        const val absFront: String = "absFront"
        const val absRight: String = "absRight"
        const val absLeft: String = "absLeft"


        // The front of the robot is where the servo hub and the battery are
        // The names are structured the following:
        // f for front, m for middle, and b for back
        // R for Right, L for Left
        // CS for Color Sensor
        // Therefore fRCS would be front, right Color Sensor
        const val frontSlotRightSensor: String = "fRCS"
        const val rightSlotRightSensor: String = "mRCS"
        const val leftSlotRightSensor: String = "bRCS"

        const val frontSlotLeftSensor: String = "fLCS"
        const val rightSlotLeftSensor: String = "mLCS"
        const val leftSlotLeftSensor: String = "bLCS"

    }

    object Positions {
        // Front slot servo: feed = 0.49, home = 0.83
        const val feedPosition: Double = 0.49
        const val homePosition: Double = 0.83
        const val awakePosition: Double = 0.01
    }
}