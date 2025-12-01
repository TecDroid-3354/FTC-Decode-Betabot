package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// After connecting via WiFi to the REV Control Hub, visit 192.168.43.1:8001 to access Panels
// This interface allows us to see the current robot position in the field according to the localizer
public class Constants {

    // Follower constants ///
    // Consists of values from the automatic, PID, and centripetal tuners
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.0) // Robot's mass in kilograms. Compensates for the centripetal force
            .forwardZeroPowerAcceleration(-45.7432) // Measures how the robot decelerates when moving forward & power is cut
            // It should be negative. Otherwise, it'll accelerate when getting closer to 0
            .lateralZeroPowerAcceleration(-52.0536) // Measures how the robot decelerates when moving sideways & power is cut
            // It should be negative. Otherwise, it'll accelerate when getting closer to 0
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.0, 0.0, 0.0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.35,0.0,0.01,0.015));


    // Drivetrain constants //
    // These contain constants specific to our drivetrain type, i.e. the Mecanum
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) // Max attainable power. Must be a number between 0 & 1
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(12.04) // The final velocity achieved by the robot after forward/backward testing
            .yVelocity(25.77); // The final velocity achieved by the robot after lateral testing


    // Localizer constants //
    // Constants specific to the localizer, i.e. the OTOS
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos") // Name of the OTOS in the hardware map
            .linearUnit(DistanceUnit.INCH) // Unit used to measure
            .angleUnit(AngleUnit.RADIANS) // Unit used to measure
            .linearScalar(1.05) // Conversion factor between OTOS reading & actual inches
            .angularScalar(0.98) // Conversion factor between OTOS reading & actual turn
            .offset(new SparkFunOTOS.Pose2D(-4.25,0.0,-110.0 * (Math.PI / 180)));
            // .offset represents the sensor's position relative to the center of the robot


    // Path constraints //
    // They determine when the path may end
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);


    // Follower creator //
    // The following method constructs the follower
    // This is the method used in the OpModes
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .OTOSLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
