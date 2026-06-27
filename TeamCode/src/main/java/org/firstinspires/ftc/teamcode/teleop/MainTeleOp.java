package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;
import org.firstinspires.ftc.teamcode.robot.utils.MirrorTool;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "Main Duo TeleOp", group = "Competition")
public class MainTeleOp extends DecodeOpMode {

    // Хардкоженные координаты, если автонома НЕ БЫЛО
    private final Pose HARDCODED_BLUE_POSE = new Pose(72, 72, 0);
    private final Pose HARDCODED_RED_POSE = new Pose(136.08, 7.458, Math.PI);

    // Цель башни
    private final Pose baseTargetPose = new Pose(144, 144, 0);

    private boolean isAutoAiming = false;
    private boolean isAutoDrivingToZone = false;
    private boolean isIntakeOn = false;

    public static double idleIntakePower = 0.1;

    @Override
    public void onInit() {
        telemetry.addLine("Ready to rumble!");

        // Показываем в INIT, какая позиция применится
        boolean isBlue = GlobalState.isBlueAlliance;
        telemetry.addData("Выбранный Альянс", isBlue ? "🟦 СИНИЙ" : "🟥 КРАСНЫЙ");
        telemetry.addData("Был ли Автоном?", GlobalState.isAutoBeen ? "ДА ✅" : "НЕТ ❌");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on();
        robot.shooter.turnOn();

        // 🔥 ЛОГИКА АВТО-ПОЗИЦИИ ПРИ СТАРТЕ ТЕЛЕОПА
        boolean isBlue = GlobalState.isBlueAlliance;
        Pose startingPose;

        // ВАЖНО: В GlobalState переменная должна называться currentPose (как ты скидывал ранее),
        // если ты ее переименовал в lastAutoPose, оставь как тут.
        if (GlobalState.isAutoBeen && GlobalState.currentPose != null) {
            startingPose = GlobalState.currentPose;
        } else {
            startingPose = isBlue ? HARDCODED_BLUE_POSE : HARDCODED_RED_POSE;
        }

        robot.drive.setPose(startingPose);
        GlobalState.isAutoBeen = false;
    }

    @Override
    public void onUpdate() {
        Pose currentPose = robot.drive.getPose();
        boolean isBlue = GlobalState.isBlueAlliance;

        // ==========================================
        // 🕹️ ГЕЙМПАД 1: ШАССИ (Водитель)
        // ==========================================

        // 1. СБРОС POSЕ НА ХАРДКОД
        if (base.wasJustPressed(GamepadKeys.Button.START)) {
            robot.drive.setPose(isBlue ? HARDCODED_BLUE_POSE : HARDCODED_RED_POSE);
            gamepad1.rumble(200);
        }

        // 2. Переключение Field-Centric
        if (base.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.drive.toggleFieldCentric();
        }

        // 🔥 3. АВТО-ПОВОРОТЫ (SNAP TURNING) - Вернул на место!

        // 4. АВТО-ПОВОРОТЫ (Динамика от альянса)
        Double snapAngle = null;
        if (isBlue) {
            // Синий альянс (без изменений)
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = Math.PI; // 180
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = Math.PI / 2; // 90
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = 0.0; // 0
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = -Math.PI / 2; // 270
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = 3 * Math.PI / 4; // 135
        } else {
            // 🔥 Обновленный красный альянс (сдвинут на -90 градусов)
            if (base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) snapAngle = 0.0; // 0
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) snapAngle = -Math.PI / 2; // -90 (или 270)
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) snapAngle = Math.PI; // 180
            else if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) snapAngle = Math.PI / 2; // 90
            else if (base.wasJustPressed(GamepadKeys.Button.B)) snapAngle = Math.PI / 4; // 45 (Диагональ Up-Right)
        }
        if (snapAngle != null) robot.drive.setSnapTarget(snapAngle);

        // 4. ИНТЕЙК ВОДИТЕЛЯ (Тогл и Реверс)
        if (base.wasJustPressed(GamepadKeys.Button.Y)) {
            isIntakeOn = !isIntakeOn;
        }

        double intakePower = isIntakeOn ? 1.0 : idleIntakePower;

        if (base.getButton(GamepadKeys.Button.A)) {
            intakePower = -1.0;
        }

        // 5. АВТО-АССИСТ (Подъезд к зоне)
        boolean hasManualInput = Math.abs(base.getLeftY()) > 0.1 ||
                Math.abs(base.getLeftX()) > 0.1 ||
                Math.abs(base.getRightX()) > 0.1 ||
                base.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        if (base.wasJustPressed(GamepadKeys.Button.X) && !isAutoDrivingToZone) {
            Pose checkPose = isBlue ? MirrorTool.toBlue(currentPose) : currentPose;

            if (PoseController.isInZone(checkPose)) {
                gamepad1.rumble(300);
            } else {
                Pose targetSnapped = PoseController.getNearestPose(checkPose);
                Pose finalTarget = isBlue ? MirrorTool.toBlue(targetSnapped) : targetSnapped;

                PathChain pathToZone = robot.drive.getFollower().pathBuilder()
                        .addPath(new BezierLine(currentPose, finalTarget))
                        .setTangentHeadingInterpolation()
                        .build();

                robot.drive.getFollower().followPath(pathToZone, true);
                isAutoDrivingToZone = true;
            }
        }

        if (isAutoDrivingToZone) {
            if (hasManualInput) {
                robot.drive.getFollower().breakFollowing();
                robot.drive.startTeleop();
                isAutoDrivingToZone = false;
            } else if (!robot.drive.getFollower().isBusy()) {
                robot.drive.startTeleop();
                isAutoDrivingToZone = false;
            }
        } else {
            robot.drive.drive(base, isBlue);
        }

        // ==========================================
        // 🎯 ГЕЙМПАД 2: ШУТЕР, БАШНЯ, ХУД (Оператор)
        // ==========================================
        if (helper.wasJustPressed(GamepadKeys.Button.B)) {
            isAutoAiming = !isAutoAiming;
        }

        Pose actualTargetPose = isBlue ? MirrorTool.toBlue(baseTargetPose) : baseTargetPose;

        if (isAutoAiming) {
            Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
            double distanceToTarget = Math.hypot(actualTargetPose.getX() - futurePose.getX(), actualTargetPose.getY() - futurePose.getY());

            robot.turret.face(actualTargetPose, futurePose);
            robot.hood.setDistance(distanceToTarget);
            robot.shooter.setDistance(distanceToTarget);
        } else {
            robot.turret.setYaw(0);
        }

        boolean isOperatorShooting = false;

        if (helper.getButton(GamepadKeys.Button.X)) {
            Pose checkPose = isBlue ? MirrorTool.toBlue(currentPose) : currentPose;
            if (PoseController.isInZone(checkPose)) {
                isOperatorShooting = true;
            } else {
                gamepad2.rumble(50);
            }
        }

        if (isOperatorShooting) {
            robot.stopper.open();
            intakePower = 1.0;
        } else {
            robot.stopper.close();
        }

        robot.intake.setPower(intakePower);

        // ==========================================
        // 📊 ТЕЛЕМЕТРИЯ
        // ==========================================
        telemetry.addData("Shooter RPM", robot.shooter.getVelocity());
        telemetry.addData("Auto-Aim", isAutoAiming ? "ON" : "OFF");

        Pose checkPose = isBlue ? MirrorTool.toBlue(currentPose) : currentPose;
        telemetry.addData("In Launch Zone?", PoseController.isInZone(checkPose) ? "YES ✅" : "NO ❌");
        telemetry.addData("Auto-Assist Driving", isAutoDrivingToZone ? "ACTIVE" : "STANDBY");
        telemetry.addData("Alliance", isBlue ? "🟦 BLUE" : "🟥 RED");
        telemetry.update();
    }
}