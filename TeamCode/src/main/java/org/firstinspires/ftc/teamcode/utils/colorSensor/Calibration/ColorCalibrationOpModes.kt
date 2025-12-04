package org.firstinspires.ftc.teamcode.utils.colorSensor.Calibration

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants.Extensions

@TeleOp(name = "ColorCalibrationOpModes")
class ColorCalibrationOpModes : LinearOpMode() {
    private lateinit var FrontSlot: ColorCalibrationJSON

    private lateinit var MiddleSlot: ColorCalibrationJSON

    private lateinit var BackSlot: ColorCalibrationJSON

    override fun runOpMode() {
        FrontSlot = ColorCalibrationJSON(IndexerConstants.Identification.FrontSlot.frontSlotLeftSensor, Extensions.frontSlotExtension, telemetry, hardwareMap, gamepad1)
        MiddleSlot = ColorCalibrationJSON(IndexerConstants.Identification.MiddleSlot.middleSlotLeftSensor, Extensions.middleSlotExtension, telemetry, hardwareMap, gamepad1)
        BackSlot = ColorCalibrationJSON(IndexerConstants.Identification.BackSlot.backSlotLeftSensor, Extensions.backSlotExtension, telemetry, hardwareMap, gamepad1)

        val opciones = listOf("FrontSlot", "MiddleSlot", "BackSlot")
        var indice = 0

        while (!isStarted && !isStopRequested) {
            if (gamepad1.dpad_left) indice = (indice - 1 + opciones.size) % opciones.size
            if (gamepad1.dpad_right) indice = (indice + 1) % opciones.size

            telemetry.addLine("Select the Slot:")
            for (i in opciones.indices) {
                if (i == indice)
                    telemetry.addLine(" ➤ ${opciones[i]}")  // seleccionado
                else
                    telemetry.addLine("   ${opciones[i]}")
            }
            telemetry.update()

            sleep(200) // evita múltiples cambios por una sola pulsación
        }

        waitForStart()

        while (opModeIsActive()) {
            when (opciones[indice]) {
                "FrontSlot" -> FrontSlot()
                "MiddleSlot" -> MiddleSlot()
                "BackSlot" -> BackSlot()
            }
        }
    }

    fun FrontSlot() {
        FrontSlot.runOpMode()
    }

    fun MiddleSlot() {
        MiddleSlot.runOpMode()
    }

    fun BackSlot() {
        BackSlot.runOpMode()
    }
}

