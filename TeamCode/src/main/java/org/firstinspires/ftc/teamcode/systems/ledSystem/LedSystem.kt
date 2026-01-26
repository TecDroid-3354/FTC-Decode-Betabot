package org.firstinspires.ftc.teamcode.systems.ledSystem

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

@Suppress("JoinDeclarationAndAssignment")
class LedSystem(hardwareMap: HardwareMap, val telemetry: Telemetry) {

    /* DECLARATION */

    // Declare the LED Controller
    private val lights: RevBlinkinLedDriver

    // Keeps track of the current displayed pattern in the LEDs
    lateinit var displayedPattern: BlinkinPattern

    /* INITIALIZATION CODE */

    init {
        // Initialize the LED controller
        lights = hardwareMap.get(RevBlinkinLedDriver::class.java, "lights")

        // Reset the LED controller
        lights.resetDeviceConfigurationForOpMode()

        // Calls an initial displayed pattern
        blinkWithPattern(BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE)
    }

    /* FUNCTIONAL CODE */

    /**
     * Sets a [BlinkinPattern] to the LED controller and updates [displayedPattern]
     * Does nothing when the requested pattern is already displayed.
     * @param pattern the desired pattern to display
     */
    fun blinkWithPattern(pattern: BlinkinPattern) {
        if (pattern == displayedPattern) {
            return
        }

        displayedPattern = pattern
        lights.setPattern(pattern)
    }

    /**
     * Call within the OpMode loop if led pattern displaying is needed
     */
    fun periodic() {
        telemetry.addData("Current LED pattern", displayedPattern)
    }
}