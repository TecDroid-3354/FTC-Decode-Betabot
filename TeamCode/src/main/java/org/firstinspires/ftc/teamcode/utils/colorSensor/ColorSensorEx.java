package org.firstinspires.ftc.teamcode.utils.colorSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ReadFile;
import org.json.JSONObject;

import java.io.File;
import java.util.HashMap;
import java.util.Map;


/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */

public class ColorSensorEx {
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    // JSON values
    Map<DetectedColor, float[]> colorCalibrations = new HashMap<>();

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        GREEN,
        PURPLE,
        WHITE,
        BLACK,
        UNKNOWN
    }

    public ColorSensorEx(ColorSensor colorSensor, Telemetry telemetry) {
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;

        loadCalibration();
    }

    public DetectedColor getColorFromSensor() {
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

            telemetry.addLine("Calibraciones cargadas desde JSON");
        } catch (Exception e) {
            telemetry.addLine("Error al cargar calibración JSON");
            telemetry.addData("Exception", e.toString());
        }
    }
}