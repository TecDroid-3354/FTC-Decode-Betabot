package org.firstinspires.ftc.teamcode.utils.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerConstants;
import org.firstinspires.ftc.teamcode.utils.ReadFile;
import org.json.JSONObject;
import java.io.File;
import java.io.FileWriter;
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor;

@TeleOp(name = "ColorSensor-calibration", group = "Op Mode")
//@Disabled
public class ColorCalibrationJSON extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, IndexerConstants.Ids.colorSensorFront);

        telemetry.addLine("Coloca el sensor frente al color y presiona:");
        telemetry.addLine("A=RED | B=BLUE | X=YELLOW | Y=WHITE");
        telemetry.addLine("Izq=GREEN | Der=PURPLE | Arriba=UNKNOWN | Abajo=BLACK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) guardarColor(DetectedColor.RED);
            if (gamepad1.b) guardarColor(DetectedColor.BLUE);
            if (gamepad1.x) guardarColor(DetectedColor.YELLOW);
            if (gamepad1.y) guardarColor(DetectedColor.WHITE);

            if (gamepad1.dpad_left) guardarColor(DetectedColor.GREEN);
            if (gamepad1.dpad_right) guardarColor(DetectedColor.PURPLE);
            if (gamepad1.dpad_up) guardarColor(DetectedColor.UNKNOWN);
            if (gamepad1.dpad_down) guardarColor(DetectedColor.BLACK);
        }
    }

    private void guardarColor(DetectedColor target) {
        // Promediamos varias lecturas
        float hueSum = 0, satSum = 0, valSum = 0;
        int samples = 20;

        for (int i = 0; i < samples; i++) {
            float[] hsv = getHSV();
            hueSum += hsv[0];
            satSum += hsv[1];
            valSum += hsv[2];
            sleep(50);
        }

        float hue = hueSum / samples;
        float sat = satSum / samples;
        float val = valSum / samples;

        telemetry.addLine("Guardando en JSON...");
        telemetry.addData("Color", target);
        telemetry.addData("Hue", hue);
        telemetry.addData("Sat", sat);
        telemetry.addData("Val", val);
        telemetry.update();

        // Guardamos en archivo JSON
        try {
            File file = new File("/sdcard/FIRST/colorCalibration.json"); // ruta en RC
            JSONObject json;

            if (file.exists()) {
                // Si ya existe, lo abrimos
                json = new JSONObject(ReadFile.readFile(file));
            } else {
                json = new JSONObject();
            }

            // Guardamos valores calibrados
            JSONObject colorData = new JSONObject();
            colorData.put("hue", hue);
            colorData.put("saturation", sat);
            colorData.put("value", val);

            json.put(target.name(), colorData);

            // Escribimos de vuelta el archivo
            FileWriter writer = new FileWriter(file);
            writer.write(json.toString(4)); // 4 espacios de indentación
            writer.close();

            telemetry.addLine("Guardado exitoso en:");
            telemetry.addData("Archivo", file.getAbsolutePath());
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Error guardando archivo");
            telemetry.addData("Exception", e.toString());
            telemetry.update();
        }

        sleep(1000);
    }

    private float[] getHSV() {
        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        float max = Math.max(r, Math.max(g, b));
        if (max == 0) max = 1;
        float rn = r / max;
        float gn = g / max;
        float bn = b / max;

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV((int)(rn * 255), (int)(gn * 255), (int)(bn * 255), hsv);

        return hsv;
    }
}
