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
import org.firstinspires.ftc.teamcode.robot.utils.MirrorTool;

@Configurable
@Autonomous(name = "🚀 Main Auto (China inspo)", group = "Autonomous")
public class MainAutoChina extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime stopperTimer;
    private ElapsedTime intakeTimer;

    // ==========================================
    // 🗺️ СТРУКТУРА КООРДИНАТ (Пишем только КРАСНЫЕ!)
    // ==========================================
    private final Pose targetPose = new Pose(144, 144, 0);

    private final Pose startPose = new Pose(118.5, 125.83, 0.7469);
    private final Pose scorePose = new Pose(88.6, 79.63, -0.51);

    private final Pose MiddleBalls = new Pose(133, 59.37, 0);
    private final Pose gateBalls = new Pose(131.41, 60.2, 0.568);
    // 🔥 Точка ПЕРЕД первым гейтом (за 10-15 дюймов до цели)
    private final Pose preGate1 = new Pose(115.0, 60.2, 0.568);

    private final Pose gateBalls2 = new Pose(131.41, 60.2, 0.568);
    // 🔥 Точка ПЕРЕД вторым гейтом
    private final Pose preGate2 = new Pose(115.0, 60.2, 0.568);
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
    public static double shootTime = 0.6;
    public static double stopperWaitTime = 0.3;
    public static double idleIntakePower = 0.2;

    // 🔥 Таймаут для защиты от застреваний (в секундах)
    public static double pathTimeout = 5.0;

    // 🔥 Настройки для умного забора с гейта
    public static double gatePickupTimeout = 3.0;
    public static double jamRpmThreshold = 500.0;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;
    // 🔥 Флажок для прелоада
    private boolean isPreloadFired = false;

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
        Pose mStart = getPose(startPose);
        Pose mScore = getPose(scorePose);
        Pose mMid1 = getPose(MiddleBalls);

        Pose mPreGate1 = getPose(preGate1);
        Pose mGate1 = getPose(gateBalls);
        Pose mPreGate2 = getPose(preGate2);
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
                .setReversed()
                .build();

        // 🔥 ОБНОВЛЕННЫЙ ПУТЬ: 1-Й ЗАЕЗД В ГЕЙТ
        grabBall2 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp2, mPreGate1))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(mPreGate1, mGate1))
                .setLinearHeadingInterpolation(mPreGate1.getHeading(), mGate1.getHeading())
                .build();

        scoreBall2 = follower.pathBuilder()
                .addPath(new BezierLine(mGate1, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // 🔥 ОБНОВЛЕННЫЙ ПУТЬ: 2-Й ЗАЕЗД В ГЕЙТ
        grabBall3 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cpGate2, mPreGate2))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(mPreGate2, mGate2))
                .setLinearHeadingInterpolation(mPreGate2.getHeading(), mGate2.getHeading())
                .build();

        scoreBall3 = follower.pathBuilder()
                .addPath(new BezierLine(mGate2, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabBall4 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp3, mFourth))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall4 = follower.pathBuilder()
                .addPath(new BezierLine(mFourth, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grabBall5 = follower.pathBuilder()
                .addPath(new BezierCurve(mScore, cp4, mNear))
                .setTangentHeadingInterpolation()
                .build();

        scoreBall5 = follower.pathBuilder()
                .addPath(new BezierLine(mNear, mScore))
                .setTangentHeadingInterpolation()
                .setReversed()
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

        robot = new Robot();
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
        Pose actualTarget = getPose(targetPose);

        // 1. ДИНАМИКА ДЛЯ СЕРВОПРИВОДОВ (Башня и Худ)
        double dynamicDistance = Math.hypot(actualTarget.getX() - currentPose.getX(), actualTarget.getY() - currentPose.getY());

        robot.turret.face(actualTarget, currentPose);
        robot.hood.setDistance(dynamicDistance);

        // 2. СТАТИКА ДЛЯ МАХОВИКА (Мотор)
        Pose actualScorePose = getPose(scorePose);
        double staticScoreDistance = Math.hypot(actualTarget.getX() - actualScorePose.getX(), actualTarget.getY() - actualScorePose.getY());

        robot.shooter.setDistance(staticScoreDistance);

        autonomousPathUpdate();

        telemetry.addData("Состояние автонома (State)", pathState);
        telemetry.addData("Башня наведена?", robot.turret.getCurrentTicks());
        telemetry.addData("Шутер готов?", robot.shooter.isAtTarget());
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

            // 🔥 Линейный сбор (MiddleBalls)
            case 4: handleLinePickup(5, scoreBall1); break;

            case 5: handleFiring(6, grabBall2); break;
            case 6: handleStopperClose(7); break;
            case 7: handleIntakeDelay(8); break;

            // 🔥 Умный сбор с гейта (gateBalls)
            case 8: handleGatePickup(9, scoreBall2, gatePickupTimeout, jamRpmThreshold); break;

            case 9: handleFiring(10, grabBall3); break;
            case 10: handleStopperClose(11); break;
            case 11: handleIntakeDelay(12); break;

            // 🔥 Умный сбор с гейта (gateBalls2)
            case 12: handleGatePickup(13, scoreBall3, gatePickupTimeout, jamRpmThreshold); break;

            case 13: handleFiring(14, grabBall4); break;
            case 14: handleStopperClose(15); break;
            case 15: handleIntakeDelay(16); break;

            // 🔥 Линейный сбор (FourthBall)
            case 16: handleLinePickup(17, scoreBall4); break;

            case 17: handleFiring(18, grabBall5); break;
            case 18: handleStopperClose(19); break;
            case 19: handleIntakeDelay(20); break;

            // 🔥 Линейный сбор (NearBalls)
            case 20: handleLinePickup(21, scoreBall5); break;

            case 21: handleFiring(22, park); break;
            case 22: handleStopperClose(999); break;

            case 999:
                robot.intake.setPower(0);
                robot.shooter.turnOff();
                if (!follower.isBusy() || pathTimer.seconds() > pathTimeout) {
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
        // 🔥 Ждем остановки шасси. Если это прелоад — ждем и маховик. Иначе лупим сразу.
        if (!follower.isBusy() && !readyToFire) {
            if (robot.shooter.isAtTarget() || isPreloadFired) {
                firingTimer.reset();
                readyToFire = true;
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
            isPreloadFired = true; // 🔥 Прелоад отстреляли, больше не ждем!

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

    // 🔥 БЫСТРЫЙ СБОР (Для линии - без остановки)
    private void handleLinePickup(int nextState, PathChain nextPath) {
        boolean isDoneDriving = !follower.isBusy();
        boolean isTimedOut = pathTimer.seconds() > pathTimeout;

        if (isDoneDriving || isTimedOut) {
            if (isTimedOut) {
                follower.breakFollowing();
            }
            stopperTimer.reset();
            follower.followPath(nextPath, true);
            setPathState(nextState);
        }
    }

    // 🔥 УМНЫЙ СБОР С ГЕЙТА (RPM и Таймер)
    private void handleGatePickup(int nextState, PathChain nextPath, double waitTimeout, double rpmThreshold) {
        boolean isDoneDriving = !follower.isBusy();
        boolean isPathTimedOut = pathTimer.seconds() > pathTimeout;

        if ((isDoneDriving || isPathTimedOut) && !arrivedAtPickup) {
            if (isPathTimedOut) {
                follower.breakFollowing();
            }
            intakeTimer.reset();
            arrivedAtPickup = true;
        }

        if (arrivedAtPickup) {
            boolean isGateTimedOut = intakeTimer.seconds() > waitTimeout;

            boolean isIntakeJammed = false;
            if (intakeTimer.seconds() > 0.5) {
                isIntakeJammed = Math.abs(robot.intake.getVelocity()) < rpmThreshold;
            }

            if (isGateTimedOut || isIntakeJammed) {
                robot.intake.setPower(idleIntakePower);
                stopperTimer.reset();
                follower.followPath(nextPath, true);
                arrivedAtPickup = false;
                setPathState(nextState);
            }
        }
    }
}