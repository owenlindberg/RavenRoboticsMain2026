package org.firstinspires.ftc.teamcode.RavenRobotics;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Under the Hive", group = "Robot")
public class MainAutonomous2 extends OpMode {
    // hardware
    DcMotorEx ramp;
    DcMotorEx intake;
    DcMotorEx shooter;
    Servo servo;

    // follower
    private Follower follower;

    // poses
    private final PoseFactory pose = PoseFactory.degrees();
    private final Pose start = pose.of(60, 9, 90);

    private final Pose path1 = pose.of(9.1925, 117.9539, 90);
    private final Pose path1Control1 = pose.of(92.5688, 138.5969, 0);
    private final Pose path1Control2 = pose.of(30.8621, 118.7508, 0);

    public Path Path1() {
        return curve(start, path1Control1, path1Control2, path1).linear(start, path1);
    }

    double timer = 0;
    private final Command shoot = Command.build()
            .setStart(() -> {
                shooter.setVelocity(2000 * 0.42);
                ramp.setPower(0.2);
                timer = 0;
            })
            .setExecute(() -> {
                if (shooter.getVelocity() > 2000 * 0.42 * 0.95) {
                    servo.setPosition(0.2);
                    ramp.setPower(0.8);
                    timer++;
                }
            })
            .setDone(() -> {
                return timer > 60 * 4;
            })
            .setEnd(endCondition -> {
                servo.setPosition(0);
                shooter.setPower(0);
                ramp.setPower(0);
                timer = 0;
            });

    private Command autoRoutine() {
        return sequential(
                shoot,
                follow(follower, Path1())
        );
    }



    @Override
    public void init() {
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        ramp.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        servo = hardwareMap.get(Servo.class, "servo");

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void start() {
        servo.setPosition(0);
        shooter.setPower(0);
        intake.setPower(0);
        ramp.setPower(0);

        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
