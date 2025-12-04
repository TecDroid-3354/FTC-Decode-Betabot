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
    private final String archiveExtension;

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

    public ColorSensorEx(ColorSensor colorSensor, String archiveExtension, Telemetry telemetry) {
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;
        this.archiveExtension = archiveExtension;

        loadCalibration();
    }

    public DetectedColor getColorFromSensor() {
        Double[] rgb = getRGB();

        DetectedColor closestColor = DetectedColor.UNKNOWN;
        double minDistance = Double.MAX_VALUE;

        // Comparamos contra cada color calibrado
        for (Map.Entry<DetectedColor, float[]> entry : colorCalibrations.entrySet()) {
            float[] ref = entry.getValue();

            // Distancia Euclidiana en espacio HSV
            double dist = Math.sqrt(
                    Math.pow(rgb[0] - ref[0], 2) +
                            Math.pow(rgb[1] - ref[1], 2) +
                            Math.pow(rgb[2] - ref[2], 2)
            );

            if (dist < minDistance) {
                minDistance = dist;
                closestColor = entry.getKey();
            }
        }

        return closestColor;
    }

    private Double[] getRGB() {
        double r = (double) colorSensor.red() / colorSensor.alpha();
        double g = (double) colorSensor.green() / colorSensor.alpha();
        double b = (double) colorSensor.blue() / colorSensor.alpha();

        return new Double[]{r, g, b};
    }

    private void loadCalibration() {
        try {
            File file = new File("/sdcard/FIRST/colorCalibration" + archiveExtension + ".json");
            JSONObject json = new JSONObject(ReadFile.readFile(file));

            for (DetectedColor c : DetectedColor.values()) {
                if (json.has(c.name())) {
                    JSONObject data = json.getJSONObject(c.name());
                    float red = (float) data.getDouble("red");
                    float green = (float) data.getDouble("green");
                    float blue = (float) data.getDouble("blue");

                    colorCalibrations.put(c, new float[]{red, green, blue});
                }
            }

            telemetry.addLine("Calibraciones cargadas desde JSON");
        } catch (Exception e) {
            telemetry.addLine("Error al cargar calibración JSON");
            telemetry.addData("Exception", e.toString());
        }
    }
}