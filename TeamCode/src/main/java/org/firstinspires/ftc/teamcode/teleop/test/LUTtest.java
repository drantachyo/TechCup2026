package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.PoseController;
import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@Configurable
@TeleOp(name = "Test: LUT Calibration", group = "Test")
public class LUTtest extends DecodeOpMode {

    // Координаты цели (корзины)
    public static double TARGET_X = 144.0;
    public static double TARGET_Y = 144.0;

    // --- РУЧНЫЕ НАСТРОЙКИ ДЛЯ КАЛИБРОВКИ LUT ---
    public static double TEST_RPM = 1200.0;
    public static double TEST_HOOD = 0.5;

    public static double INTAKE_POWER = 0.8;

    @Override
    public void onInit() {
        telemetry.addLine("Режим калибровки LUT готов.");
        telemetry.addLine("Меняй TEST_RPM и TEST_HOOD в Panels!");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on();
        robot.shooter.turnOn();
        robot.stopper.close();
    }

    @Override
    public void onUpdate() {
        robot.drive.drive(base, true);

        // ==========================================
        // 1. РАСЧЕТЫ И ПОЗИЦИОНИРОВАНИЕ
        // ==========================================
        Pose targetPose = new Pose(TARGET_X, TARGET_Y, 0);
        Pose currentPose = robot.drive.getPose();
        Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
        double angleVel = robot.drive.getFollower().getAngularVelocity();

        // Считаем дистанцию (для записи в таблицу)
        double distanceToTarget = currentPose.distanceFrom(targetPose);

        // ==========================================
        // 2. РУЧНОЕ УПРАВЛЕНИЕ МЕХАНИЗМАМИ (БЕЗ АВТОМАТИКИ)
        // ==========================================
        // Жестко задаем значения из дашборда, игнорируя дистанцию
        robot.shooter.setTargetVelocity(TEST_RPM);
        robot.hood.setPosition(TEST_HOOD);

        // Башню оставляем автоматической, чтобы она всегда смотрела на цель
        robot.turret.face(targetPose, futurePose, angleVel);

        // ==========================================
        // 3. УПРАВЛЕНИЕ ИНТЕЙКОМ И СТОППЕР
        // ==========================================
        if (base.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intake.setPower(INTAKE_POWER);
        } else if (base.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intake.setPower(-INTAKE_POWER);
        } else {
            robot.intake.setPower(0);
        }

        // Выстрел
        if (base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            if (robot.shooter.isAtTarget()) {
                robot.stopper.open();
            }
        } else {
            robot.stopper.close();
        }

        // ==========================================
        // 4. ТЕЛЕМЕТРИЯ ДЛЯ ЗАПИСИ ДАННЫХ
        // ==========================================
        telemetry.addData("=== ЗАПИШИ ЭТИ ДАННЫЕ В LUT ===", "");
        telemetry.addData("-> DISTANCE", distanceToTarget);
        telemetry.addData("-> RPM", TEST_RPM);
        telemetry.addData("-> HOOD POS", TEST_HOOD);

        telemetry.addLine(" ");
        telemetry.addData("Target X", TARGET_X);
        telemetry.addData("Target Y", TARGET_Y);
        telemetry.addData("Real RPM", robot.shooter.getVelocity());
        telemetry.update();
    }
}