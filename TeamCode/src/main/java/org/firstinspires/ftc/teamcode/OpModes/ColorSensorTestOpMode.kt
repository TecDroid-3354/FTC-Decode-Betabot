package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants

/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */
@Disabled
@TeleOp(name = "ColorSensorTest", group = "Op Mode")
class ColorSensorTestOpMode : CommandOpMode() {
    private lateinit var colorSensor: ColorSensor

    override fun initialize() {
        /* Subsystem initialization */
        colorSensor = hardwareMap.get(ColorSensor::class.java, IndexerConstants.Identification.FrontSlot.frontSlotLeftSensorId)

        configureButtonBindings()
    }

    fun configureButtonBindings() {

    }

    private fun getRGB(): DoubleArray {
        val r = colorSensor.red().toDouble() / colorSensor.alpha()
        val g = colorSensor.green().toDouble() / colorSensor.alpha()
        val b = colorSensor.blue().toDouble() / colorSensor.alpha()

        return doubleArrayOf(r, g, b)
    }

    fun periodic() {
        telemetry.addData("r", getRGB()[0])
        telemetry.addData("g", getRGB()[1])
        telemetry.addData("b", getRGB()[2])
    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {

            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()
            periodic()

            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}