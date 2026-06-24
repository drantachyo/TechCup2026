package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@TeleOp(name = "Test: Turret Only", group = "Test")
public class TurretTest extends DecodeOpMode {

    // Тестовый угол (в градусах для удобства восприятия)
    private double testAngleDegrees = 0;

    // Шаг поворота башни на одно нажатие
    private final double stepDegrees = 15.0;

    @Override
    public void onInit() {
        telemetry.addLine("System: Turret Test Ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine(" - D-Pad LEFT/RIGHT: Rotate Turret by 15 deg");
        telemetry.addLine(" - X (Square): Emergency ZERO (Set angle to 0)");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on(); // Включаем ПИД башни
    }

    @Override
    public void onUpdate() {
        // 1. Управление целевым углом
        if (base.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            testAngleDegrees += stepDegrees;
        } else if (base.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            testAngleDegrees -= stepDegrees;
        }

        // 2. Экстренный сброс в центр (если башня закрутилась)
        if (base.wasJustPressed(GamepadKeys.Button.X)) {
            testAngleDegrees = 0;
        }

        // 3. Отправляем угол в башню (переводим градусы в радианы)
        robot.turret.setYaw(Math.toRadians(testAngleDegrees));

        // ==========================================
        // ТЕЛЕМЕТРИЯ (Смотреть в FTC Dashboard)
        // ==========================================
        telemetry.addData("1. Target Angle (Deg)", testAngleDegrees);
        telemetry.addData("2. Current Angle (Deg)", Math.toDegrees(robot.turret.getYaw()));
        telemetry.addData("3. Target Ticks", Turret.targetTicks);
        telemetry.addData("4. Current Ticks", robot.turret.getCurrentTicks());
        telemetry.addData("5. Error (Ticks)", Turret.targetTicks - robot.turret.getCurrentTicks());
        telemetry.update();
    }
}