package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@TeleOp(name = "Tuning: Shooter + Hood LUT", group = "Test")
public class ShooterHoodTuning extends DecodeOpMode {

    // Координаты корзины (обязательно проверь, совпадают ли они с твоим полем!)
    private final Pose targetPose = new Pose(144, 144, 0);

    // Стартовые тестовые значения
    private double testRpm = 1500;
    private double testHoodPos = 0.5;

    @Override
    public void onInit() {
        telemetry.addLine("System: LUT Tuning Ready");
        telemetry.addLine("Driver: Drive around and shoot (Right Trigger)");
        telemetry.addLine("Helper: D-Pad UP/DOWN (RPM), LEFT/RIGHT (Hood)");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.shooter.turnOn();
        robot.turret.on(); // Включаем башню для автоприцела
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // 1. ДРАЙВЕР (Gamepad 1): Езда и выстрел
        // ==========================================
        robot.drive.drive(base, true); // Катаемся (Blue Alliance)

        // Башня автоматически всегда смотрит на цель, чтобы мы стреляли прямо
        robot.turret.face(targetPose, robot.drive.getPose(), robot.drive.getFollower().getAngularVelocity());

        // Выстрел (Допустим, интейк закидывает кольцо в шутер по триггеру)
        if (base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            robot.intake.setPower(1.0);
        } else {
            robot.intake.setPower(0);
        }

        // ==========================================
        // 2. ПОМОЩНИК (Gamepad 2): Настройка параметров
        // ==========================================

        // Настраиваем RPM шагом по 50
        if (helper.wasJustPressed(GamepadKeys.Button.DPAD_UP)) testRpm += 50;
        if (helper.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) testRpm -= 50;

        // Настраиваем Худ шагом по 0.02
        if (helper.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) testHoodPos += 0.02;
        if (helper.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) testHoodPos -= 0.02;

        // Отправляем тестовые данные в механизмы
        robot.shooter.setTargetVelocity(testRpm);
        robot.hood.setPosition(testHoodPos);

        // ==========================================
        // 3. МАТЕМАТИКА (Высчитываем дистанцию глазами Pedro Pathing)
        // ==========================================
        Pose currentPose = robot.drive.getPose();

        // Считаем дистанцию от центра робота до корзины по теореме Пифагора (Math.hypot)
        double calculatedDistance = Math.hypot(
                targetPose.getX() - currentPose.getX(),
                targetPose.getY() - currentPose.getY()
        );

        // ==========================================
        // 4. ВЫВОД ДАННЫХ ДЛЯ ЗАПИСИ
        // ==========================================
        telemetry.addLine("=== WRITE THIS TO YOUR CODE ===");
        // Форматируем дистанцию до 1 знака после запятой для красоты
        telemetry.addData("Distance", "%.1f", calculatedDistance);
        telemetry.addData("Shooter RPM", testRpm);
        telemetry.addData("Hood Pos", "%.2f", testHoodPos);
        telemetry.addLine("===============================");

        // Контроль того, что мотор реально успел разогнаться перед выстрелом
        telemetry.addData("Real RPM", robot.shooter.getVelocity());
        telemetry.update();
    }
}