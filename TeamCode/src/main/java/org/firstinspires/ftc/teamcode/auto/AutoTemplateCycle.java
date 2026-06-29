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
@Autonomous(name = "🔥 Template: 3 Cycles Tries", group = "Autonomous")
public class AutoTemplateCycle extends OpMode {

    private Robot robot;
    private Follower follower;

    private int pathState;
    private ElapsedTime pathTimer;
    private ElapsedTime autoTimer;

    // ==========================================
    // 📍 КООРДИНАТЫ (Подставь свои)
    // ==========================================
    private final Pose startPose = new Pose(56.6, 7.2761,  Math.PI);
    private final Pose targetPose = new Pose(0, 144, 0); // 🥅 Цель для башни
    private final Pose endPose = new Pose(30, 30, Math.PI); // Парковка

    // 🎯 Дальняя зона стрельбы
    private final Pose farShootPose = new Pose(60.3, 12.34, Math.PI);

    // ➖ Линия (напольный сэмпл)
    private final Pose lineIntakePose = new Pose(10.4, 36.4, Math.PI);
    public static Pose controlPointLine = new Pose(30.0, 30.0);

    // 📦 Дальняя зона забора (Submersible / Gate)
    private final Pose farIntakeTry1 = new Pose(10, 8, Math.PI);

    // 🔥 Точка отката (Тот же Y и Heading, но X на 15 больше)
    private final Pose farIntakeBackup = new Pose(farIntakeTry1.getX() + 15, farIntakeTry1.getY(), farIntakeTry1.getHeading());

    // ==========================================
    // ⏱️ НАСТРОЙКИ ТАЙМИНГОВ
    // ==========================================
    public static double shootTime = 0.7;
    public static double stabilizationTime = 0.2;

    // 🔥 Время на первоначальную раскрутку маховика (в секундах от старта матча)
    public static double spinUpTime = 2.0;

    public static double autoTimeLimit = 28.0;

    // 🔥 Время на один трай (вмазаться и пожевать)
    public static double tryTimeout = 2.0;

    // 🔥 Время на откат назад (быстрое движение)
    public static double backupTimeout = 1.0;

    // Пути
    private PathChain scorePreload;
    private PathChain driveToLine, scoreLineElement;
    private PathChain driveToIntakeTry1, driveToBackup, driveToIntakeTry2, scoreZoneElement;
    private PathChain park;

    public void buildPaths() {
        // 💡 СОВЕТ: Если робот едет к какой-то точке "не тем концом" (задом вместо переда),
        // просто добавь .setReversed(true) перед .build() в нужном пути!

        scorePreload = follower.pathBuilder().addPath(new BezierLine(startPose, farShootPose)).setLinearHeadingInterpolation(startPose.getHeading(), farShootPose.getHeading()).build();

        driveToLine = follower.pathBuilder().addPath(new BezierLine(farShootPose, lineIntakePose)).setLinearHeadingInterpolation(farShootPose.getHeading(), lineIntakePose.getHeading()).build();
        scoreLineElement = follower.pathBuilder().addPath(new BezierCurve(lineIntakePose, controlPointLine, farShootPose)).setLinearHeadingInterpolation(lineIntakePose.getHeading(), farShootPose.getHeading()).build();

        // Пути для цикла тарана
        driveToIntakeTry1 = follower.pathBuilder().addPath(new BezierLine(farShootPose, farIntakeTry1)).setLinearHeadingInterpolation(farShootPose.getHeading(), farIntakeTry1.getHeading()).build();
        driveToBackup = follower.pathBuilder().addPath(new BezierLine(farIntakeTry1, farIntakeBackup)).setLinearHeadingInterpolation(farIntakeTry1.getHeading(), farIntakeBackup.getHeading()).build();
        driveToIntakeTry2 = follower.pathBuilder().addPath(new BezierLine(farIntakeBackup, farIntakeTry1)).setLinearHeadingInterpolation(farIntakeBackup.getHeading(), farIntakeTry1.getHeading()).build();

        // Если при возврате на скор робот должен ехать задом, раскомментируй setReversed(true):
        scoreZoneElement = follower.pathBuilder().addPath(new BezierLine(farIntakeTry1, farShootPose)).setLinearHeadingInterpolation(farIntakeTry1.getHeading(), farShootPose.getHeading())/* .setReversed(true) */.build();

        park = follower.pathBuilder().addPath(new BezierLine(farShootPose, endPose)).setLinearHeadingInterpolation(farShootPose.getHeading(), endPose.getHeading()).build();
    }

    @Override
    public void init() {
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
        robot.shooter.turnOn(); // Шутер начинает раскручиваться СРАЗУ
        autoTimer.reset();
        setPathState(10);
    }

    @Override
    public void loop() {
        robot.periodic();
        Pose currentPose = follower.getPose();

        if (autoTimer.seconds() >= autoTimeLimit && pathState < 998) {
            robot.intake.setPower(0);
            robot.stopper.close();
            follower.breakFollowing();
            follower.followPath(park, true);
            setPathState(998);
        }

        if (pathState < 998) {
            double dynamicDist = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
            robot.turret.face(targetPose, currentPose);
            robot.hood.setDistance(dynamicDist);

            double staticDist = Math.hypot(targetPose.getX() - farShootPose.getX(), targetPose.getY() - farShootPose.getY());
            robot.shooter.setDistance(staticDist);
        } else {
            robot.turret.setYaw(0);
        }

        // 🔥 ВОТ ЭТА СТРОЧКА У ТЕБЯ ПОТЕРЯЛАСЬ! Без неё стейты не переключаются.
        autonomousPathUpdate();
    }

    public void autonomousPathUpdate() {
        boolean isPathDone = !follower.isBusy();

        switch (pathState) {
            // ==========================================
            // 🎯 ФАЗА 1: ПРЕЛОАД И ЛИНИЯ
            // ==========================================
            case 10:
                robot.stopper.close();
                robot.intake.setPower(0);
                follower.followPath(scorePreload, true);
                setPathState(11);
                break;

            case 11:
                if (isPathDone) {
                    // 🔥 Мы доехали. Проверяем маховик:
                    if (autoTimer.seconds() < spinUpTime) {
                        // Маховик еще крутится. Ждем, не пускаем таймер выстрела!
                        pathTimer.reset();
                    } else {
                        // Маховик готов! Начинаем секвенцию выстрела
                        if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                        if (pathTimer.seconds() > stabilizationTime + shootTime) {
                            robot.stopper.close();
                            follower.followPath(driveToLine, true);
                            setPathState(12);
                        }
                    }
                } else {
                    // Мы еще в пути, держим таймер на нуле
                    pathTimer.reset();
                }
                break;

            case 12:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    follower.followPath(scoreLineElement, true);
                    setPathState(13);
                } else { pathTimer.reset(); }
                break;

            case 13:
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveToIntakeTry1, true);
                        setPathState(20);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🔁 ЦИКЛ 1
            // ==========================================
            case 20: // ТРАЙ 1: Вмазываемся
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToBackup, true);
                    setPathState(21);
                }
                break;

            case 21: // ОТКАТ НАЗАД
                robot.intake.setPower(1.0);
                if (isPathDone || pathTimer.seconds() > backupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToIntakeTry2, true);
                    setPathState(22);
                }
                break;

            case 22: // ТРАЙ 2: Снова вмазываемся
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreZoneElement, true);
                    setPathState(23);
                }
                break;

            case 23: // Скор Цикла 1
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveToIntakeTry1, true);
                        setPathState(30);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🔁 ЦИКЛ 2
            // ==========================================
            case 30: // ТРАЙ 1
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToBackup, true);
                    setPathState(31);
                }
                break;

            case 31: // ОТКАТ
                robot.intake.setPower(1.0);
                if (isPathDone || pathTimer.seconds() > backupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToIntakeTry2, true);
                    setPathState(32);
                }
                break;

            case 32: // ТРАЙ 2
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreZoneElement, true);
                    setPathState(33);
                }
                break;

            case 33: // Скор Цикла 2
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(driveToIntakeTry1, true);
                        setPathState(40);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🔁 ЦИКЛ 3
            // ==========================================
            case 40: // ТРАЙ 1
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToBackup, true);
                    setPathState(41);
                }
                break;

            case 41: // ОТКАТ
                robot.intake.setPower(1.0);
                if (isPathDone || pathTimer.seconds() > backupTimeout) {
                    follower.breakFollowing();
                    follower.followPath(driveToIntakeTry2, true);
                    setPathState(42);
                }
                break;

            case 42: // ТРАЙ 2
                robot.intake.setPower(1.0);
                if (pathTimer.seconds() > tryTimeout) {
                    follower.breakFollowing();
                    follower.followPath(scoreZoneElement, true);
                    setPathState(43);
                }
                break;

            case 43: // Скор Цикла 3
                robot.intake.setPower(1.0);
                if (isPathDone) {
                    if (pathTimer.seconds() > stabilizationTime) { robot.stopper.open(); }
                    if (pathTimer.seconds() > stabilizationTime + shootTime) {
                        robot.stopper.close();
                        follower.followPath(park, true); // Все 3 цикла выполнены, едем на парковку
                        setPathState(998);
                    }
                } else { pathTimer.reset(); }
                break;

            // ==========================================
            // 🛑 ПАРКОВКА И ФИНАЛ
            // ==========================================
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
}