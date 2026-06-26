package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;
import org.firstinspires.ftc.teamcode.teleop.DecodeOpMode;

@TeleOp(name = "Test: Turret Auto-Aim Math", group = "Test")
public class TurretMathTest extends DecodeOpMode {

    // Координаты цели (например, корзина)
    private final Pose targetPose = new Pose(144, 144, 0);

    @Override
    public void onInit() {
        telemetry.addLine("System: Turret Math Test Ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine(" - Drive around normally");
        telemetry.addLine(" - HOLD RIGHT_BUMPER to physically aim the turret");
        telemetry.update();
    }

    @Override
    public void onStart() {
        robot.turret.on(); // Включаем ПИД (чтобы башня могла держать 0 градусов)
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // 1. УПРАВЛЕНИЕ ШАССИ
        // ==========================================
        robot.drive.drive(base, true);

        // ==========================================
        // 2. ПОЛУЧЕНИЕ ПРЕДСКАЗАНИЙ ИЗ PEDRO PATHING
        // ==========================================
        Pose futureRobotPose = PoseController.getFuturePose(robot.drive.getFollower());
        double angleVel = robot.drive.getFollower().getAngularVelocity();

        // ==========================================
        // 3. ФОНОВЫЙ РАСЧЕТ ОЖИДАЕМОГО УГЛА (Для телеметрии)
        // ==========================================
        // Повторяем твою логику из Turret.java
        double y = targetPose.getY() - futureRobotPose.getY();
        double x = targetPose.getX() - futureRobotPose.getX();

        double calcAngleVel = angleVel;
        if (Math.abs(calcAngleVel) < 2.5) {
            calcAngleVel = 0;
        }

        // Чистый глобальный угол + упреждение скорости
        double rawGlobalAngle = Math.atan2(y, x) + (calcAngleVel * Turret.rAM);

        // Перевод в локальный угол башни
        double expectedLocalAngleRadians = rawGlobalAngle - futureRobotPose.getHeading();
        expectedLocalAngleRadians = Turret.normalizeAngle(expectedLocalAngleRadians);

        // ==========================================
        // 4. ТЕСТИРОВАНИЕ КНОПКОЙ
        // ==========================================
        if (base.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            // КНОПКА ЗАЖАТА: Отдаем управление автоприцелу
            robot.turret.face(targetPose, futureRobotPose, angleVel);
            telemetry.addLine(">>> STATUS: TURRET IS TRACKING! <<<");
        } else {
            // КНОПКА ОТПУЩЕНА: Возвращаем башню строго вперед
            robot.turret.setYaw(0);
            telemetry.addLine(">>> STATUS: DRIVING (Turret locked at 0) <<<");
        }

        // ==========================================
        // 5. ТЕЛЕМЕТРИЯ (Смотри в Dashboard)
        // ==========================================
        telemetry.addData("1. EXPECTED Angle (Deg)", Math.toDegrees(expectedLocalAngleRadians));
        telemetry.addData("2. EXPECTED Ticks", (int)(expectedLocalAngleRadians * Turret.ticksPerRadian));
        telemetry.addLine("---");
        telemetry.addData("3. Real Target Ticks", Turret.targetTicks);
        telemetry.addData("4. Real Current Ticks", robot.turret.getCurrentTicks());
        telemetry.update();
    }
}