package org.firstinspires.ftc.teamcode.subsystems.indexer

class IndexerConstants {

    object Identification {

        // These are the Ids of each of the component per slot, they must be called in the robot configuraation
        // inside the Control HUB

        // The front of the robot is where the servo hub and the battery are
        // The names of the color sensors are structured the following:
        // f for front, m for middle, and b for back
        // R for Right, L for Left
        // CS for Color Sensor
        // Therefore fRCS would be front, right Color Sensor

        object FrontSlot {

            const val frontServoId: String = "frontServo"

            const val frontSlotRightSensorId: String = "fRCS"

            const val frontSlotLeftSensorId: String = "fLCS"
        }

        object MiddleSlot {

            const val middleServoId: String = "middleServo"

            const val middleSlotRightSensorId: String = "mRCS"

            const val middleSlotLeftSensorId: String = "mLCS"
        }
        object BackSlot {

            const val backServoId: String = "backServo"

            const val backSlotRightSensorId: String = "bRCS"

            const val backSlotLeftSensorId: String = "bLCS"
        }

    }

    object Configuration {

        // Gives each servo an inverted value
        const val isFrontServoInverted: Boolean = true

        const val isMiddleServoInverted: Boolean = false

        const val isBackServoInverted: Boolean = false
    }

    object Positions {
        // The HOME, and FEED positions for each servo, these are passed through the slot Config and grabbed by
        // the Slot class
        object FrontPositions {
            const val FEED: Double = 0.3

            const val HOME: Double = 0.01
        }

        object MiddlePositions {
            const val FEED: Double = 0.3

            const val HOME: Double = 0.0
        }

        object BackPositions {
            const val FEED: Double = 0.3

            const val HOME: Double = 0.0
        }
    }

    object ColorRanges{

        val purpleHUERange = 200.0..300.0

        val greenSATRange = 0.5..0.70
    }

    object Extensions {

        // These are useful for configuring each slot color detection and tune them separately
        const val frontSlotExtension: String = "FrontSlot"

        const val middleSlotExtension: String = "MiddleSlot"

        const val backSlotExtension: String = "BackSlot"
    }
}