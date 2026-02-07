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
            .mass(15.0) // Robot's mass. Compensates for the centripetal force
            .forwardZeroPowerAcceleration(-29.7404) // 👍 Measures how the robot decelerates when moving forward & power is cut
            // It should be negative. Otherwise, it'll accelerate when getting closer to 0
            .lateralZeroPowerAcceleration(-68.9878) //👍 Measures how the robot decelerates when moving sideways & power is cut
            // It should be negative. Otherwise, it'll accelerate when getting closer to 0
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0.0, 0.0, 0.0)) //👍
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.37,0.0,0.01,0.015)) //👍
            .centripetalScaling(0.0002);


    // Drivetrain constants //
    // These contain constants specific to our drivetrain type, i.e. the Mecanum
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) // Max attainable power. Must be a number between 0 & 1
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(56.5904) // 👍 The final velocity achieved by the robot after forward/backward testing
            .yVelocity(24.3991); // The final velocity achieved by the robot after lateral testing


    // Localizer constants //
    // Constants specific to the localizer, i.e. the OTOS
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos") // Name of the OTOS in the hardware map
            .linearUnit(DistanceUnit.INCH) // Unit used to measure
            .angleUnit(AngleUnit.RADIANS) // Unit used to measure
            // .offset(x, y, h) represents the sensor's position relative to the center of the robot
            // The sensor is aligned along a normal cartesian plane's x axis. It is horizontally
            // translated 1/4 in to the left, which would be a translation in the y axis since OTOS
            // axes are flipped.
//            .offset(new SparkFunOTOS.Pose2D(0.0,-0.25,-Math.PI / 2.0))
            //.offset(new SparkFunOTOS.Pose2D(0.0, -0.70637, Math.PI / 2))
            .offset(new SparkFunOTOS.Pose2D(0.0, -0.70637, -110.0 * (Math.PI / 180)))
            //.offset(new SparkFunOTOS.Pose2D(0.0, -0.70637, Math.PI / 2)) // works with translational & rotational tuning


            //.offset(new SparkFunOTOS.Pose2D(0.0, -0.70637, -Math.PI))
            .linearScalar(1.1658) // Done 👍
            .angularScalar(0.9898); // Done 👍


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
