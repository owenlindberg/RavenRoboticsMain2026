package org.firstinspires.ftc.teamcode.RavenRobotics;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class MainTeleOp extends OpMode {
    // hardware
    DcMotorEx intake;
    DcMotorEx shooter;
    NormalizedColorSensor topColorSensor;
    Servo hoodServo;
//    Limelight3A limelight;

    // follower
    private Follower follower;

    // poses
    private final PoseFactory pose = PoseFactory.degrees();
    private final Pose start = pose.of(72,72,90);
    private Pose curr = pose.of(0,0,90);
    private final Pose end = pose.of(72,72,90);

    // paths
    private Path Park() {
        return line(curr, end).linear(curr, end);
    }

    WaverlyGamepad gp = null;


    double intakePower = 0.4;
    double shooterPower = 0.4;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        topColorSensor = hardwareMap.get(NormalizedColorSensor.class, "topColor");

        hoodServo = hardwareMap.get(Servo.class, "HoodServo");

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();
//        limelight.pipelineSwitch(0);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        gp = new WaverlyGamepad(gamepad1);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        gp.readButtons();

        // driving & parking
        if (gp.xToggled) {
//            curr = follower.pose();
//            if (Math.abs(curr.x() - end.x()) > 1/2d || Math.abs(curr.y() - end.y()) > 1/2d) {
//                schedule(park());
//                Scheduler.execute();
//            }
            follower.hold(end, true);
        }
        else {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.manual(forward, lateral, turn);
        }
        follower.update();


        // intake
        if (gp.aToggled) {
            intake.setVelocity(2000 * (gp.yToggled? -intakePower : intakePower));
        } else {
            intake.setVelocity(0);
        }
        if (gp.dpadUpPressed) {
            intakePower = Math.min(intakePower + 0.05, 1);
        } else if (gp.dpadDownPressed) {
            intakePower = Math.max(intakePower - 0.05, 0);
        }


        // shooter motor
        if (gp.bToggled) {
            shooter.setVelocity(2000 * shooterPower);
        } else {
            shooter.setVelocity(0);
        }
        if (gp.rightBumperPressed) {
            shooterPower = Math.min(shooterPower + 0.05, 1);
        } else if (gp.leftBumperPressed) {
            shooterPower = Math.max(shooterPower - 0.05, 0);
        }


        // hood servo
        if (gp.dpadLeft) {
            hoodServo.setPosition(1);
        } else {
            hoodServo.setPosition(0);
        }


        // color sensor
        NormalizedRGBA colors = topColorSensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());

        String detectedColor;
        if (hue >= 0 && hue <= 80) {
            detectedColor = "RED";
        }
        else if (hue >= 220 && hue <= 250) {
            detectedColor = "BLUE";
        }
        else if (hue >= 90 && hue <= 110) {
            detectedColor = "YELLOW";
        }
        else {
            detectedColor = "NONE";
        }


        // telemetry
        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addLine("Intake speed at " + intakePower*100 + "%");
        telemetry.addLine("Shooter speed at " + shooterPower*100 + "%");

        telemetry.addLine("\nButtons:");
        telemetry.addLine("a (x) = toggle intake on/off");
        telemetry.addLine("x (sq) = toggle hold position");
        telemetry.addLine("y (tri) = toggle intake direction");
        telemetry.addLine("b (circ) = toggle shooter on/off");

        telemetry.addLine("\nColor Sensor Output:");

        telemetry.addData("Light Detected", ((OpticalDistanceSensor) topColorSensor).getLightDetected());
        telemetry.addData("Red", colors.red);
        telemetry.addData("Green", colors.green);
        telemetry.addData("Blue", colors.blue);
        telemetry.addLine("\ncolor: " + detectedColor + "\n");
        telemetry.addData("Hue", hue);
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        telemetry.update();
    }
}
