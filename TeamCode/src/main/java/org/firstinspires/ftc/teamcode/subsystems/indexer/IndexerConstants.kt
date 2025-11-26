package org.firstinspires.ftc.teamcode.subsystems.indexer

class IndexerConstants {
    object Ids {
        const val frontServo: String = "frontServo"
        const val rightServo: String = "leftServo"
        const val leftServo: String = "rightServo"

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
        const val middleSlotRightSensor: String = "mRCS"
        const val backSlotRightSensor: String = "bRCS"

        const val frontSlotLeftSensor: String = "fLCS"
        const val middleSlotLeftSensor: String = "mLCS"
        const val backSlotLeftSensor: String = "bLCS"

    }

    object Positions {

        object frontPositions {

            const val feedPosition: Double = 0.49

            const val homePosition: Double = 0.83
        }

        object middlePositions {

            const val feedPosition: Double = 0.28

            const val homePosition: Double = 0.6
        }

        object backPositions {

            const val feedPosition: Double = 0.88

            const val homePosition: Double = 0.56
        }
    }

    object Extensions {
        // Front slot servo: feed = 0.49, home = 0.83
        const val frontSlotExtension: String = "FrontSlot"
        const val middleSlotExtension: String = "MiddleSlot"
        const val backSlotExtension: String = "BackSlot"
    }
}