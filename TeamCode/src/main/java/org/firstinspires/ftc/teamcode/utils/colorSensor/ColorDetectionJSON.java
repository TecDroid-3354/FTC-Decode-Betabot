package org.firstinspires.ftc.teamcode.utils.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.utils.ReadFile;
import org.json.JSONObject;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.utils.colorSensor.ColorSensorEx.DetectedColor;
@Disabled
@TeleOp(name = "ColorSensor-Test", group = "Op Mode")

public class ColorDetectionJSON extends LinearOpMode {

    ColorSensor colorSensor;

    // JSON values
    Map<DetectedColor, float[]> colorCalibrations = new HashMap<>();

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Cargar calibraciones desde archivo
        loadCalibration();

        telemetry.addLine("Sistema de detección listo.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            DetectedColor detected = getColorFromSensor();
            telemetry.addData("Color detectado", detected);
            telemetry.update();
        }
    }

    private void loadCalibration() {
        try {
            File file = new File("/sdcard/FIRST/colorCalibration.json");
            JSONObject json = new JSONObject(ReadFile.readFile(file));

            for (DetectedColor c : DetectedColor.values()) {
                if (json.has(c.name())) {
                    JSONObject data = json.getJSONObject(c.name());
                    float hue = (float) data.getDouble("hue");
                    float sat = (float) data.getDouble("saturation");
                    float val = (float) data.getDouble("value");

                    colorCalibrations.put(c, new float[]{hue, sat, val});
                }
            }

            telemetry.addLine("✅ Calibraciones cargadas desde JSON");
        } catch (Exception e) {
            telemetry.addLine("❌ Error al cargar calibración JSON");
            telemetry.addData("Exception", e.toString());
        }
    }

    private DetectedColor getColorFromSensor() {
        float[] hsv = getHSV();

        DetectedColor closestColor = DetectedColor.UNKNOWN;
        double minDistance = Double.MAX_VALUE;

        // Comparamos contra cada color calibrado
        for (Map.Entry<DetectedColor, float[]> entry : colorCalibrations.entrySet()) {
            float[] ref = entry.getValue();

            // Distancia Euclidiana en espacio HSV
            double dist = Math.sqrt(
                    Math.pow(hsv[0] - ref[0], 2) +
                            Math.pow(hsv[1] - ref[1], 2) +
                            Math.pow(hsv[2] - ref[2], 2)
            );

            if (dist < minDistance) {
                minDistance = dist;
                closestColor = entry.getKey();
            }
        }

        return closestColor;
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
        android.graphics.Color.RGBToHSV(
                (int)(rn * 255),
                (int)(gn * 255),
                (int)(bn * 255),
                hsv
        );

        return hsv; // [hue, sat, val]
    }
}
