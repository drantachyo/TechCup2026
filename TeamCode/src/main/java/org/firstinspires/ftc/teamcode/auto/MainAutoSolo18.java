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
import org.firstinspires.ftc.teamcode.robot.utils.MirrorTool; // 🔥 Не забудь этот импорт!

@Configurable
@Autonomous(name = "🚀 Main Auto (Универсальный)", group = "Autonomous")
public class MainAutoSolo18 extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime stopperTimer;
    private ElapsedTime intakeTimer;
    private ElapsedTime scoreStabilizeTimer;

    // ==========================================
    // 🗺️ СТРУКТУРА КООРДИНАТ (Пишем только КРАСНЫЕ!)
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);

    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    private final Pose MiddleBalls = new Pose(133, 59.37, 0);
    private final Pose gateBalls = new Pose(131.41, 60.2, 0.568);
    private final Pose gateBalls2 = new Pose(131.41, 60.2, 0.568);
    private final Pose FourthBall = new Pose(132, 36.7, 0);
    private final Pose NearBalls = new Pose(127, 83.25, 0);

    private final Pose endPose = new Pose(100, 60, 0);

    // Контрольные точки
    public static Pose controlPoint1 = new Pose(90, 59);
    public static Pose controlPoint2 = new Pose(100, 59);
    public static Pose controlPointGate2 = new Pose(100, 59);
    public static Pose controlPoint3 = new Pose(100, 36);
    public static Pose controlPoint4 = new Pose(100, 80);

    // ==========================================
    // ⏱️ ТАЙМИНГИ И МОЩНОСТИ
    // ==========================================
    public static double shootTime = 0.7;
    public static double stopperWaitTime = 0.3;
    public static double intakeTime = 0.3;
    public static double pickupWaitTime = 0.5;
    public static double scoreWaitTime = 0.15;
    public static double idleIntakePower = 0.2;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;
    private boolean isStabilizing = false;

    // Пути Pedro Pathing
    private PathChain scorePreload;
    private PathChain grabBall1, scoreBall1;
    private PathChain grabBall2, scoreBall2;
    private PathChain grabBall3, scoreBall3;
    private PathChain grabBall4, scoreBall4;
    private PathChain grabBall5, scoreBall5;
    private PathChain park;

    // ==========================================
    // 🪞 АВТО-ЗЕРКАЛО КООРДИНАТ
    // ==========================================
    private Pose getPose(Pose redPose) {
        if (GlobalState.isBlueAlliance) {
            return MirrorTool.toBlue(redPose);
        }
        return redPose;
    }

    // ==========================================
    // 🛣️ СТРУКТУРА ПУТЕЙ
    // ==========================================
    public void buildPaths() {
        // 🔥 Автоматически конвертируем все точки под выбранный альянс перед созданием путей
        Pose mStart = getPose(startPose);
        Pose mScore = getPose(scorePose);
        Pose mMid1 = getPose(MiddleBalls);
        Pose mGate1 = getPose(gateBalls);
        Pose mGate2 = getPose(gateBalls2);
        Pose mFourth = getPose(FourthBall);
        Pose mNear = getPose(NearBalls);
        Pose mEnd = getPose(endPose);

        Pose cp1 = getPose(controlPoint1);
        Pose cp2 = getPose(controlPoint2);
        Pose cpGate2 = getPose(controlPointGate2);
        Pose cp3 = getPose(controlPoint3);
        Pose cp4 = getPose(controlPoint4);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(mStart, mScore))
                .setLinearHeadingInterpolation(mStart.getHeading(), mScore.getHeading())
                .build();

        grabBall1 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp1, mMid1))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall1 = follower.pathBuilder()
                .addPath(new BezierLine(mMid1, mScore))
                .setTangentHeadingInterpolation()
                .setReversed(true) // 🔥 Исправлено на true
                .build();

        grabBall2 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp2, mGate1))
                .setLinearHeadingInterpolation(mScore.getHeading(), mGate1.getHeading())
                .build();

        scoreBall2 = follower.pathBuilder()
                .addPath(new BezierLine(mGate1, mScore))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        grabBall3 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cpGate2, mGate2))
                .setLinearHeadingInterpolation(mScore.getHeading(), mGate2.getHeading())
                .build();

        scoreBall3 = follower.pathBuilder()
                .addPath(new BezierLine(mGate2, mScore))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        grabBall4 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp3, mFourth))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall4 = follower.pathBuilder()
                .addPath(new BezierLine(mFourth, mScore))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        grabBall5 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp4, mNear))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall5 = follower.pathBuilder()
                .addPath(new BezierLine(mNear, mScore))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(mScore, mEnd))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        firingTimer = new ElapsedTime();
        stopperTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        scoreStabilizeTimer = new ElapsedTime();

        robot = new Robot();
        // 🔥 Инициализируем робота в правильной стартовой точке
        robot.init(hardwareMap, getPose(startPose));
        follower = robot.drive.getFollower();

        buildPaths();

        telemetry.addLine("Универсальный Автоном Готов!");
        telemetry.addData("Альянс", GlobalState.isBlueAlliance ? "🟦 СИНИЙ" : "🟥 КРАСНЫЙ");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.turret.on();
        robot.shooter.turnOn();
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.periodic();

        Pose currentPose = follower.getPose();

        // 🔥 Считаем цель для башни с учетом альянса
        Pose actualTarget = getPose(targetPose);
        double distanceToTarget = Math.hypot(actualTarget.getX() - currentPose.getX(), actualTarget.getY() - currentPose.getY());

        robot.turret.face(actualTarget, currentPose);
        robot.hood.setDistance(distanceToTarget);
        robot.shooter.setDistance(distanceToTarget);

        autonomousPathUpdate();

        telemetry.addData("Состояние автонома (State)", pathState);
        telemetry.addData("Башня наведена?", robot.turret.getCurrentTicks());
        telemetry.update();
    }

    @Override
    public void stop() {
        GlobalState.currentPose = follower.getPose();
        GlobalState.isAutoBeen = true;
    }

    // ==========================================
    // 🧠 КОНЕЧНЫЙ АВТОМАТ (STATE MACHINE)
    // ==========================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.stopper.close();
                robot.intake.setPower(idleIntakePower);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: handleFiring(2, grabBall1); break;
            case 2: handleStopperClose(3); break;
            case 3: handleIntakeDelay(4); break;
            case 4: handlePickupWait(5, scoreBall1, getPose(MiddleBalls), pickupWaitTime); break;
            case 5: handleFiring(6, grabBall2); break;
            case 6: handleStopperClose(7); break;
            case 7: handleIntakeDelay(8); break;
            case 8: handlePickupWait(9, scoreBall2, getPose(gateBalls), pickupWaitTime * 3.5); break;
            case 9: handleFiring(10, grabBall3); break;
            case 10: handleStopperClose(11); break;
            case 11: handleIntakeDelay(12); break;
            case 12: handlePickupWait(13, scoreBall3, getPose(gateBalls2), pickupWaitTime * 4.0); break;
            case 13: handleFiring(14, grabBall4); break;
            case 14: handleStopperClose(15); break;
            case 15: handleIntakeDelay(16); break;
            case 16: handlePickupWait(17, scoreBall4, getPose(FourthBall), pickupWaitTime); break;
            case 17: handleFiring(18, grabBall5); break;
            case 18: handleStopperClose(19); break;
            case 19: handleIntakeDelay(20); break;
            case 20: handlePickupWait(21, scoreBall5, getPose(NearBalls), pickupWaitTime); break;
            case 21: handleFiring(22, park); break;
            case 22: handleStopperClose(999); break;

            case 999:
                robot.intake.setPower(0);
                robot.shooter.turnOff();
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    // ==========================================
    // 🛠️ ХЕЛПЕР-МЕТОДЫ (ЛОГИКА ОПЕРАЦИЙ)
    // ==========================================

    private void handleFiring(int nextState, PathChain nextPath) {
        if (!follower.isBusy() && !isStabilizing && !readyToFire) {
            isStabilizing = true;
            scoreStabilizeTimer.reset();
        }

        if (isStabilizing && scoreStabilizeTimer.seconds() > scoreWaitTime) {
            if (robot.shooter.isAtTarget()) {
                firingTimer.reset();
                readyToFire = true;
                isStabilizing = false;
            }
        }

        if (readyToFire) {
            robot.stopper.open();
            robot.intake.setPower(1.0);
        }

        if (readyToFire && firingTimer.seconds() > shootTime) {
            robot.stopper.close();
            robot.intake.setPower(idleIntakePower);
            follower.followPath(nextPath, true);
            readyToFire = false;
            stopperTimer.reset();
            setPathState(nextState);
        }
    }

    private void handleStopperClose(int nextState) {
        if (stopperTimer.seconds() > stopperWaitTime) {
            stopperTimer.reset();
            setPathState(nextState);
        }
    }

    private void handleIntakeDelay(int nextState) {
        if (stopperTimer.seconds() > stopperWaitTime) {
            robot.intake.setPower(1.0);
            intakeTimer.reset();
            setPathState(nextState);
        }
    }

    private void handlePickupWait(int nextState, PathChain nextPath, Pose currentTarget, double waitTime) {
        if (!follower.isBusy() && !arrivedAtPickup) {
            intakeTimer.reset();
            arrivedAtPickup = true;
        }

        if (arrivedAtPickup && intakeTimer.seconds() > waitTime) {
            robot.intake.setPower(idleIntakePower);
            stopperTimer.reset();
            follower.followPath(nextPath, true);
            arrivedAtPickup = false;
            setPathState(nextState);
        }
    }
}