package org.firstinspires.ftc.teamcode.RavenRobotics;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class TestAuto4 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.5, 8.5, 90);
    private final Pose sweepHive = poseFactory.of(58.6893, 118.4881, 270);
    private final Pose sweepHiveSegment1Start = poseFactory.of(58.6893, 118.4881, 90);
    private final Pose sweepHiveSegment1End = poseFactory.of(58.6893, 118.4881, 90);
    private final Pose sweepHiveSegment2Start = poseFactory.of(58.6893, 118.4881, 90);
    private final Pose sweepHiveSegment2End = poseFactory.of(58.6893, 118.4881, 270);
    private final Pose eatFlower1 = poseFactory.of(47.5, 133, 90);
    private final Pose eatFlower1Control1 = poseFactory.of(48.5522, 98.9449, 0);
    private final Pose goToShoot = poseFactory.of(58.5522, 118.4545, 270);
    private final Pose goToShootControl1 = poseFactory.of(48.5455, 98.9039, 0);
    private final Pose eatFlower2 = poseFactory.of(8.5, 47, 180);
    private final Pose eatFlower2Control1 = poseFactory.of(24.0216, 106.5358, 0);
    private final Pose eatFlower2Control2 = poseFactory.of(36.4165, 58.3748, 0);
    private final Pose park = poseFactory.of(11.1766, 95.357, 0);
    private final Pose parkControl1 = poseFactory.of(32.8711, 40.8577, 0);

    public Path sweepHive() {
        return line(start, sweepHive).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(sweepHiveSegment1Start, sweepHiveSegment1End)).until(1, Interpolator.linear(sweepHiveSegment2Start, sweepHiveSegment2End)));
    }

    public Path eatFlower1() {
        return curve(sweepHive, eatFlower1Control1, eatFlower1).linear(sweepHive, eatFlower1);
    }

    public Path goToShoot() {
        return curve(eatFlower1, goToShootControl1, goToShoot).linear(eatFlower1, goToShoot);
    }

    public Path eatFlower2() {
        return curve(goToShoot, eatFlower2Control1, eatFlower2Control2, eatFlower2).linear(goToShoot, eatFlower2);
    }

    public Path park() {
        return curve(eatFlower2, parkControl1, park).linear(eatFlower2, park);
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, sweepHive()),
                follow(follower, eatFlower1()),
                follow(follower, goToShoot()),
                follow(follower, eatFlower2()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void start() {
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
//        if (follower.currentPath() != null) {
//            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//            telemetry.addData("Path number", follower.pathIndex());
//        }
        telemetry.update();
    }
}
