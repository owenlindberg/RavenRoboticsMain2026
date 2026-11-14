package org.firstinspires.ftc.teamcode.RavenRobotics;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TempTeleOp extends OpMode {

    // hardware
    DcMotorEx ramp;
    DcMotorEx intake;
    DcMotorEx shooter;
    Servo servo;
    Limelight3A limelight;

    // follower
    private Follower follower;

    // poses
    private final PoseFactory pose = PoseFactory.degrees();
    private final Pose start = pose.of(72,72,0);
    private final Pose end = pose.of(72,72,90);

    WaverlyGamepad gp = null;


    double shooterPower = 0.55;

    @Override
    public void init() {
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        ramp.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        servo = hardwareMap.get(Servo.class, "servo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        gp = new WaverlyGamepad(gamepad1);
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        gp.readButtons();

        // limelight
        limelight.updateRobotOrientation(follower.pose().heading());

        LLResult result = limelight.getLatestResult();
//        Pose3D LLPose = null;
        double turnForce = 0;
        if (result != null && result.isValid()) {
//            LLPose = result.getBotpose_MT2();
            double tx = result.getTx();
            if (Math.abs(tx) < 1) {
                turnForce = tx/20;
                turnForce = Math.max(Math.min(turnForce,1),-1);
            }
            turnForce *= 0.5;
        }


        // driving & parking
        if (gp.dpadLeftToggled) {
            follower.hold(end, true);
        } else {
            if (gp.dpadRightToggled) {
                DrivePowers fieldCentricPowers = ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x + (gp.leftTrigger? turnForce : 0),
                        follower.pose().heading()
                );
                follower.manual(fieldCentricPowers);
            } else {
                follower.manual(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x + (gp.leftTrigger? turnForce : 0)
                );
            }
        }
        follower.update();


        // intake
        if (gp.aToggled) {
            intake.setPower(0.4 * (gp.dpadDownToggled ? -1 : 1));
        } else {
            intake.setVelocity(0);
        }

        // ramp
        if (gp.bToggled) {
            ramp.setPower((gp.rightTrigger ? 0.8 : 0.2) * (gp.dpadDownToggled ? -1 : 1));
        } else {
            ramp.setVelocity(0);
        }

        // servo
        if (gp.rightTrigger) {
            servo.setPosition(0.25);
        } else {
            servo.setPosition(0);
        }

        // shooter
        if (gp.yToggled) {
            shooter.setVelocity(2000 * shooterPower);
        } else {
            shooter.setPower(0);
        } if (gp.rightBumperPressed) {
            shooterPower = Math.round(Math.min(shooterPower + 0.05, 1));
        } else if (gp.leftBumperPressed) {
            shooterPower = Math.round(Math.max(shooterPower - 0.05, 0));
        }


        // telemetry
        telemetry.addLine("Shooter target speed " + Math.round(shooterPower*100) + "%");
        telemetry.addLine("Shooter status: " + ((shooter.getVelocity()/2000)/shooterPower > 0.95? "ready" : "spinning up"));
        telemetry.addLine("");
        telemetry.addLine("Limelight angle read is " + result.getTx());
        telemetry.addLine("Turn force at " + turnForce);

        Pose robotPose = follower.pose();
        telemetry.addLine("\nFollower:");
        telemetry.addLine("Robot X " + robotPose.x());
        telemetry.addLine("Robot Y " + robotPose.y());
        telemetry.addLine("Robot Heading " + Math.toDegrees(robotPose.heading()));

//        telemetry.addLine("\nLimelight:");
//        if (LLPose != null) {
//            telemetry.addLine("Robot X " + LLPose.getPosition().toUnit(DistanceUnit.INCH).x);
//            telemetry.addLine("Robot Y " + LLPose.getPosition().toUnit(DistanceUnit.INCH).y);
//            telemetry.addLine("Robot Heading " + LLPose.getOrientation().getYaw(AngleUnit.DEGREES));
//        } else {
//            telemetry.addLine("Robot X NaN");
//            telemetry.addLine("Robot Y NaN");
//            telemetry.addLine("Robot Heading NaN");
//        }

        telemetry.addLine("\nButtons:");
        telemetry.addLine("a (x) = toggle intake on/off");
        telemetry.addLine("x (sq) = ");
        telemetry.addLine("y (tri) = toggle shooter wheel spining");
        telemetry.addLine("b (circ) = toggle ramp");
        telemetry.addLine("Dpad right = toggle field-centric mode");
        telemetry.addLine("Dpad left = toggle hold position");
        telemetry.addLine("Dpad down = toggle intake & ramp direction");
        telemetry.addLine("left bumper = increase shooter speed");
        telemetry.addLine("right bumper = decrease shooter speed");
        telemetry.addLine("left trigger = the PID");
        telemetry.addLine("right trigger = shoot");

        telemetry.update();
    }
}