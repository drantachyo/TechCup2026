package org.firstinspires.ftc.teamcode.teleop.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.PoseController;

@TeleOp(name = "🧪 Тест Геозон (PoseController)", group = "Tests")
public class PoseControllerTest extends OpMode {

    private Robot robot;
    private GamepadEx driver;

    @Override
    public void init() {
        robot = new Robot();

        // Задаем стартовую позицию для теста (например, центр поля)
        // Обязательно поставь робота в эту точку перед нажатием INIT!
        robot.init(hardwareMap, new Pose(72, 72, 0));

        driver = new GamepadEx(gamepad1);

        telemetry.addLine("Тест геозон готов.");
        telemetry.addLine("Поставь робота на (72, 72) и нажми PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.drive.startTeleop();
    }

    @Override
    public void loop() {
        // Обязательный апдейт Pedro Pathing
        robot.periodic();
        driver.readButtons();

        // Езда шасси (только водитель)
        robot.drive.drive(driver, true);

        // ==========================================
        // 🛠️ ТЕСТИРОВАНИЕ POSE CONTROLLER
        // ==========================================
        Pose currentPose = robot.drive.getPose();

        // Проверяем, находится ли текущая позиция внутри твоих зон (треугольников)
        boolean inZone = PoseController.isInZone(currentPose);

        // Получаем точку "примагничивания" к зоне (если мы вне ее)
        Pose snappedPose = PoseController.getNearestPose(currentPose);

        // ==========================================
        // 📊 ВЫВОД В ТЕЛЕМЕТРИЮ
        // ==========================================
        telemetry.addLine("=== ПОЗИЦИЯ РОБОТА ===");
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Угол", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));

        telemetry.addLine("\n=== ГЕОЗОНИРОВАНИЕ ===");
        if (inZone) {
            telemetry.addData("СТАТУС", "✅ В ЗОНЕ СТРЕЛЬБЫ");
        } else {
            telemetry.addData("СТАТУС", "❌ ВНЕ ЗОНЫ");
            telemetry.addData("Ближайшая точка зоны (Snap X)", String.format("%.2f", snappedPose.getX()));
            telemetry.addData("Ближайшая точка зоны (Snap Y)", String.format("%.2f", snappedPose.getY()));
        }

        telemetry.addLine("\n=== ПРЕДСКАЗАНИЕ (rVM = " + PoseController.rVM + ") ===");
        try {
            Pose futurePose = PoseController.getFuturePose(robot.drive.getFollower());
            telemetry.addData("Future X", String.format("%.2f", futurePose.getX()));
            telemetry.addData("Future Y", String.format("%.2f", futurePose.getY()));
        } catch (Exception e) {
            telemetry.addData("Future Pose", "Ожидание расчета скорости...");
        }

        telemetry.update();
    }
}