package org.firstinspires.ftc.teamcode.utils.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/*
 * USEFUL VIDEO LINKS
 *
 *   Setting up limelight environment: https://www.youtube.com/watch?v=slt0fIq-a2E
 *   Detecting MetaTag2 & coding LimeLight + IMU: https://www.youtube.com/watch?v=-EfOzB_A00Q
 *
 *
 *   Otro:
 *      IMU / Gyro: https://www.youtube.com/watch?v=8mB7x6SNUKo
 *      Webcam: https://www.youtube.com/watch?v=OZt33z-lyYo
 * */


@TeleOp(name = "LLTuned", group = "testing")
public class LLTestTuned extends OpMode {
    private LimelightVision limelight;

    @Override
    public void init() {
        limelight = new LimelightVision(hardwareMap);
    }

    @Override
    public void start() {
        // We start the limelight specifically at this point so that it doesn't take any energy
        // before the start button is pressed in match
        limelight.enableLimelight();
    }

    @Override
    public void loop() {
        telemetry.addData("TargetDistanceMeters", limelight.getDistance().component1().getMeters());
        }
    }



    /*
     * ESTIMATING DISTANCE
     *
     *   1. Place the robot at a fixed, measured distance from the AprilTag
     *   2. Get the how big the AprilTag looks from the camera field of view, i.e. the Ta param
     *   3. Get a curve / regression out of all values
     *   4. Work backwards and from the curve, get the distance to the current point
     *
     * */
