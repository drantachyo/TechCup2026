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
@Autonomous(name = "🚀 Скелет Автонома (Танк + Удержание)", group = "Autonomous")
public class autoRed extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime stopperTimer;
    private ElapsedTime intakeTimer;
    private ElapsedTime scoreStabilizeTimer;

    // ==========================================
    // 🗺️ СТРУКТУРА КООРДИНАТ
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);

    // 1. START POSE
    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);

    // 2. SCORE POSE (Позиция с которой робот стреляет)
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    // 3. MIDDLE BALLS
    private final Pose MiddleBalls = new Pose(133, 59.37, 0);
    private final Pose gateBalls = new Pose(131.41, 60.2, 0.568);
    private final Pose gateBalls2 = new Pose(131.41, 60.2, 0.568); // Если нужно проехать глубже

    // 4. ЧЕТВЕРТЫЙ МЯЧ
    private final Pose FourthBall = new Pose(132, 36.7, 0);

    // 🔥 5. ПЯТЫЙ МЯЧ (NearBalls)
    private final Pose NearBalls = new Pose(127, 83.25, 0); // Впиши свои координаты

    // 6. КОНЕЦ (Позиция парковки)
    private final Pose endPose = new Pose(100, 60, 0);

    // Контрольные точки
    public static Pose controlPoint1 = new Pose(90, 59);
    public static Pose controlPoint2 = new Pose(100, 59);
    public static Pose controlPointGate2 = new Pose(100, 59);
    public static Pose controlPoint3 = new Pose(100, 36);
    // 🔥 Новая контрольная точка для 5-го мяча
    public static Pose controlPoint4 = new Pose(100, 80);

    // ==========================================
    // ⏱️ ТАЙМИНГИ И МОЩНОСТИ
    // ==========================================
    public static double shootTime = 0.7;
    public static double stopperWaitTime = 0.3;
    public static double intakeTime = 0.3;
    public static double pickupWaitTime = 0.5;
    public static double scoreWaitTime = 0.15;

    // Постоянная мощность интейка для удержания мячей внутри
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
    private PathChain grabBall5, scoreBall5; // 🔥 Пути для 5-го мяча (NearBall)
    private PathChain park;

    // ==========================================
    // 🛣️ СТРУКТУРА ПУТЕЙ
    // ==========================================
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabBall1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint1, MiddleBalls))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall1 = follower.pathBuilder()
                .addPath(new BezierLine(MiddleBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabBall2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint2, gateBalls))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateBalls.getHeading())
                .build();

        scoreBall2 = follower.pathBuilder()
                .addPath(new BezierLine(gateBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabBall3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPointGate2, gateBalls2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateBalls2.getHeading())
                .build();

        scoreBall3 = follower.pathBuilder()
                .addPath(new BezierLine(gateBalls2, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabBall4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint3, FourthBall))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall4 = follower.pathBuilder()
                .addPath(new BezierLine(FourthBall, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 ПУТИ ДЛЯ 5-ГО МЯЧА (NearBall)
        grabBall5 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint4, NearBalls))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall5 = follower.pathBuilder()
                .addPath(new BezierLine(NearBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
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
        robot.init(hardwareMap, startPose);
        follower = robot.drive.getFollower();

        buildPaths();

        telemetry.addLine("Шаблон готов! Ждем старта.");
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
        double distanceToTarget = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());

        robot.turret.face(targetPose, currentPose);
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

            case 1:
                handleFiring(2, grabBall1);
                break;

            case 2:
                handleStopperClose(3);
                break;

            case 3:
                handleIntakeDelay(4);
                break;

            case 4:
                handlePickupWait(5, scoreBall1, MiddleBalls, pickupWaitTime);
                break;

            case 5:
                handleFiring(6, grabBall2);
                break;

            case 6:
                handleStopperClose(7);
                break;

            case 7:
                handleIntakeDelay(8);
                break;

            case 8:
                handlePickupWait(9, scoreBall2, gateBalls, pickupWaitTime * 3.5);
                break;

            case 9:
                handleFiring(10, grabBall3);
                break;

            case 10:
                handleStopperClose(11);
                break;

            case 11:
                handleIntakeDelay(12);
                break;

            case 12:
                handlePickupWait(13, scoreBall3, gateBalls2, pickupWaitTime * 4);
                break;

            case 13:
                handleFiring(14, grabBall4);
                break;

            case 14:
                handleStopperClose(15);
                break;

            case 15:
                handleIntakeDelay(16);
                break;

            case 16:
                handlePickupWait(17, scoreBall4, FourthBall, pickupWaitTime);
                break;

            // 🔥 Выстрел 4-го мяча и едем за 5-м (NearBall)
            case 17:
                handleFiring(18, grabBall5);
                break;

            case 18:
                handleStopperClose(19);
                break;

            case 19:
                handleIntakeDelay(20);
                break;

            // 🔥 Сбор 5-го мяча (NearBall)
            case 20:
                handlePickupWait(21, scoreBall5, NearBalls, pickupWaitTime);
                break;

            // 🔥 Выстрел 5-го мяча и переход к парковке
            case 21:
                handleFiring(22, park);
                break;

            case 22:
                handleStopperClose(999);
                break;

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