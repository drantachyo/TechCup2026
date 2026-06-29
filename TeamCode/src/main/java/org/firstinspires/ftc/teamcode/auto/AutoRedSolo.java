package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;

@Configurable
@Autonomous(name = "🟥 Auto Solo 18 - RED", group = "Autonomous")
public class AutoRedSolo extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;
    private ElapsedTime autoTimer;
    private ElapsedTime pathTimer;

    // ==========================================
    // 📍 КООРДИНАТЫ ДЛЯ КРАСНОЙ СТОРОНЫ
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);
    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    private final Pose NearBalls = new Pose(127, 83.25, 0);
    private final Pose MiddleBalls = new Pose(133, 58.37, 0);

    private final Pose gateOpen1 = new Pose(125.9, 65.32, 0);
    private final Pose gatePickup1 = new Pose(132.56, 57, 0.655);

    private final Pose gateOpen2 = new Pose(125.9, 65.32, 0);
    private final Pose gatePickup2 = new Pose(132.56, 57, 0.655);

    private final Pose FourthBall = new Pose(133, 39, 0);
    private final Pose endPose = new Pose(100, 45, 0);

    public static Pose controlPoint1 = new Pose(90, 59);
    public static Pose controlPoint2 = new Pose(100, 59);
    public static Pose controlPointGate2 = new Pose(100, 59);
    public static Pose controlPoint3 = new Pose(85, 37);

    // ==========================================
    // ⏱️ НАСТРОЙКИ
    // ==========================================
    public static double shootTime = 0.9;
    public static double preFireDistance = 15.0;
    public static double pathTimeout = 5.0;
    public static double stabilizationTime = 0.25;
    public static double autoTimeLimit = 28.0;

    // 🔥 Просто стоим 2.5 секунды на гейте
    public static double gatePickupTimeout = 2.5;

    private PathChain scorePreload, park;
    private PathChain grabBall1, scoreBall1;
    private PathChain grabBall2, scoreBall2;
    private PathChain driveGateOpen1, driveGatePickup1, scoreBall3;
    private PathChain driveGateOpen2, driveGatePickup2, scoreBall4;
    private PathChain grabBall5, scoreBall5;

    public void buildPaths() {
        scorePreload = follower.pathBuilder().addPath(new BezierLine(startPose, scorePose)).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()).build();

        grabBall1 = follower.pathBuilder().addPath(new BezierLine(scorePose, NearBalls)).setTangentHeadingInterpolation().build();
        scoreBall1 = follower.pathBuilder().addPath(new BezierLine(NearBalls, scorePose)).setTangentHeadingInterpolation().setReversed().build();

        grabBall2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint1, MiddleBalls)).setTangentHeadingInterpolation().build();
        scoreBall2 = follower.pathBuilder().addPath(new BezierLine(MiddleBalls, scorePose)).setTangentHeadingInterpolation().setReversed().build();

        driveGateOpen1 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint2, gateOpen1)).setLinearHeadingInterpolation(scorePose.getHeading(), gateOpen1.getHeading()).build();
        driveGatePickup1 = follower.pathBuilder().addPath(new BezierLine(gateOpen1, gatePickup1)).setLinearHeadingInterpolation(gateOpen1.getHeading(), gatePickup1.getHeading()).build();
        scoreBall3 = follower.pathBuilder().addPath(new BezierLine(gatePickup1, scorePose)).setTangentHeadingInterpolation().setReversed().build();

        driveGateOpen2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPointGate2, gateOpen2)).setLinearHeadingInterpolation(scorePose.getHeading(), gateOpen2.getHeading()).build();
        driveGatePickup2 = follower.pathBuilder().addPath(new BezierLine(gateOpen2, gatePickup2)).setLinearHeadingInterpolation(gateOpen2.getHeading(), gatePickup2.getHeading()).build();
        scoreBall4 = follower.pathBuilder().addPath(new BezierLine(gatePickup2, scorePose)).setTangentHeadingInterpolation().setReversed().build();

        grabBall5 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint3, FourthBall)).setTangentHeadingInterpolation().build();
        scoreBall5 = follower.pathBuilder().addPath(new BezierLine(FourthBall, scorePose)).setTangentHeadingInterpolation().setReversed().build();

        park = follower.pathBuilder().addPath(new BezierLine(scorePose, endPose)).setTangentHeadingInterpolation().build();
    }

    @Override
    public void init() {
        GlobalState.isBlueAlliance = false; // 🔥 Жестко задаем для Телеопа

        pathTimer = new ElapsedTime();
        autoTimer = new ElapsedTime();
        robot = new Robot();
        robot.init(hardwareMap, startPose);
        follower = robot.drive.getFollower();
        buildPaths();
    }

    @Override
    public void start() {
        robot.turret.on();
        robot.shooter.turnOn();
        setPathState(10);
        autoTimer.reset();
    }

    @Override
    public void loop() {
        robot.periodic();
        Pose currentPose = follower.getPose();

        if (autoTimer.seconds() >= autoTimeLimit && pathState < 998) {
            robot.intake.setPower(0);
            robot.stopper.close();
            follower.followPath(park, true);
            setPathState(998);
        }

        if (pathState < 998) {
            double dynamicDist = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
            robot.turret.face(targetPose, currentPose);
            robot.hood.setDistance(dynamicDist);

            double staticDist = Math.hypot(targetPose.getX() - scorePose.getX(), targetPose.getY() - scorePose.getY());
            robot.shooter.setDistance(staticDist);
        } else {
            robot.turret.setYaw(0);
        }

        autonomousPathUpdate(currentPose);
    }

    public void autonomousPathUpdate(Pose currentPose) {
        boolean isPathDone = !follower.isBusy() || pathTimer.seconds() > pathTimeout;

        switch (pathState) {
            case 10:
                robot.stopper.close();
                robot.intake.setPower(0.8);
                follower.followPath(scorePreload, true);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) {
                        robot.stopper.open();
                        robot.intake.setPower(1.0);
                    }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(grabBall1, true);
                        setPathState(20);
                    }
                } else { pathTimer.reset(); }
                break;

            case 20:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreBall1, true);
                    setPathState(21);
                }
                break;
            case 21:
                robot.intake.setPower(1.0);
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(grabBall2, true);
                        setPathState(30);
                    }
                } else { pathTimer.reset(); }
                break;

            case 30:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreBall2, true);
                    setPathState(31);
                }
                break;
            case 31:
                robot.intake.setPower(1.0);
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveGateOpen1, true);
                        setPathState(40);
                    }
                } else { pathTimer.reset(); }
                break;

            case 40:
                robot.intake.setPower(0);
                if (isPathDone) {
                    follower.followPath(driveGatePickup1, true);
                    setPathState(42);
                }
                break;

            case 42:
                // 🔥 Просто крутим интейк и ждем таймера
                robot.intake.setPower(1.0);

                if (pathTimer.seconds() > gatePickupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreBall3, true);
                    setPathState(43);
                }
                break;

            case 43:
                robot.intake.setPower(1.0);
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveGateOpen2, true);
                        setPathState(50);
                    }
                } else { pathTimer.reset(); }
                break;

            case 50:
                robot.intake.setPower(0);
                if (isPathDone) {
                    follower.followPath(driveGatePickup2, true);
                    setPathState(52);
                }
                break;

            case 52:
                // 🔥 Просто крутим интейк и ждем таймера
                robot.intake.setPower(1.0);

                if (pathTimer.seconds() > gatePickupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreBall4, true);
                    setPathState(53);
                }
                break;

            case 53:
                robot.intake.setPower(1.0);
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(grabBall5, true);
                        setPathState(60);
                    }
                } else { pathTimer.reset(); }
                break;

            case 60:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreBall5, true);
                    setPathState(61);
                }
                break;
            case 61:
                robot.intake.setPower(1.0);
                if (!follower.isBusy()) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        robot.intake.setPower(0);
                        follower.followPath(park, true);
                        setPathState(998);
                    }
                } else { pathTimer.reset(); }
                break;

            case 998:
                robot.shooter.turnOff();
                robot.intake.setPower(0);

                if (isPathDone) {
                    setPathState(999);
                }
                break;

            case 999:
                robot.shooter.turnOff();
                robot.intake.setPower(0);

                if (pathTimer.seconds() > 0.2) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void stop() {
        GlobalState.currentPose = follower.getPose();
        GlobalState.lastAutoPose = follower.getPose();
        GlobalState.isAutoBeen = true;
    }
}