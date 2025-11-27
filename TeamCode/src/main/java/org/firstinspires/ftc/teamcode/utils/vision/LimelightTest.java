package org.firstinspires.ftc.teamcode.utils.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

@Disabled
@TeleOp(name = "limelightTest", group = "testing")
public class LimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu; // Setting the IMU is necessary to get MetaTag2,
    // which allows for better data retrieving using the IMU
    private SparkFunOTOS otos;

    private double limelightMountAngleDegrees; // LL mount angle from horizontal
    private double limelightLensHeightInches; // LL lens Height from ground
    private double goalHeightInches; // Target height from ground

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // Retrieves pipeline
        limelight.pipelineSwitch(1); // Gets the limelight pipeline

        // TODO: Get these values
        limelightMountAngleDegrees = 0.0;
        limelightLensHeightInches = 0.0;
        goalHeightInches = 0.0;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.resetTracking();
    }

    @Override
    public void start() {
        // We start the limelight specifically at this point so that it doesn't take any energy
        // before the start button is pressed in match
        limelight.start();
    }

    @Override
    public void loop() {
        // Getting the robot's orientation through the IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("orientationIMU", orientation.getYaw());
        telemetry.addData("orientationOTOS", otos.getPosition().h);

        // Updating limelights' robot orientation with the Yaw
        limelight.updateRobotOrientation(otos.getPosition().h);

        // LLResult is like a container full of information about what Limelight sees
        LLResult llResult = limelight.getLatestResult();

        // Math to calculate distance was taken from documentation:
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-area-to-estimate-distance
        // The condition verifies whether the LimeLight Result is a valid statement
        if (llResult != null && llResult.isValid()) {
            // Offset to target in degrees (from crosshair)
            double targetOffsetAngle_Vertical = llResult.getTy();
            // Needs to be in radians for tan() method
            double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);

            // Calculated distance from limelight lens to goal (in inches)
            double distanceFromLimelightToGoalInches =
                    (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            telemetry.addData("TargetDistanceInches", distanceFromLimelightToGoalInches);

            // We will first get a (MetaTag2) Pose3D. From here, we will extract its Tx, Ty & Ta components
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx()); // Represents how far left/right the target is (in degrees)
            telemetry.addData("Ty", llResult.getTy()); // Represents how far up/down the target is (in degrees)
            telemetry.addData("Ta", llResult.getTa()); // Represents how big the AprilTag looks
            // according to the camera field of view (0-100%)

            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());

            /*
             * It is important to notice that the Full3D option should be enabled
             * */
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

}
