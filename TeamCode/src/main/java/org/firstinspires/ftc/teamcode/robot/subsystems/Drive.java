package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class Drive {
    private Follower follower;
    private boolean isFieldCentric = true;
    private boolean wasBraking = false; // Флаг для режима "вкопаться"

    // ПИД для авто-поворота на углы (Snap-to-Angle)
    public static double turnP = 1.2, turnI = 0.0, turnD = 0.1;

    private PIDController snapTurnController;
    private Double snapTargetAngle = null;

    public void init(HardwareMap hw) {
        follower = Constants.createFollower(hw);
        snapTurnController = new PIDController(turnP, turnI, turnD);
    }

    public void startTeleop() {
        follower.startTeleopDrive();
    }

    public void periodic() {
        follower.update();
    }

    // ==========================================
    // ЛОГИКА УПРАВЛЕНИЯ
    // ==========================================

    public void drive(GamepadEx gamepad, boolean isBlueAlliance) {
        // 🔥 1. ПРОВЕРКА РЕЖИМОВ (Тормоз и Слоу-мод)
        boolean isBraking = gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        double currentSpeedMult = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 0.5 : 1.0;

        if (isBraking) {
            if (!wasBraking) {
                holdCurrentPosition(); // Вкапываемся в землю (Pedro Pathing PID)
                wasBraking = true;
            }
            snapTargetAngle = null;
            return; // Игнорируем любые движения стиков, пока зажат тормоз!
        } else {
            if (wasBraking) {
                startTeleop(); // Возвращаем обычный режим езды
                wasBraking = false;
            }
        }

        // 2. Считываем стики (Кубическая зависимость для плавности)
        double forward = Math.pow(gamepad.getLeftY(), 3) * currentSpeedMult;
        double strafe = Math.pow(-gamepad.getLeftX(), 3) * currentSpeedMult;
        double turn = Math.pow(-gamepad.getRightX(), 3) * currentSpeedMult * 0.7;

        // 3. Отмена авто-поворота при ручном вмешательстве
        if (Math.abs(turn) > 0.05) {
            snapTargetAngle = null;
        }

        // 4. АВТО-ПОВОРОТ (Snap-to-Angle)
        if (snapTargetAngle != null) {
            snapTurnController.setPID(turnP, turnI, turnD);
            double currentHeading = follower.getPose().getHeading();
            double error = snapTargetAngle - currentHeading;

            // Нормализуем ошибку в пределы от -PI до PI
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error <= -Math.PI) error += 2 * Math.PI;

            // 🔥 МАГИЯ D-КОМПОНЕНТА: Текущая позиция = -error, Цель = 0
            turn = snapTurnController.calculate(-error, 0);

            // Ограничиваем мощность поворота, чтобы мотор не сошел с ума
            turn = Math.max(-1.0, Math.min(1.0, turn));
        }

        // 5. Передаем мощности на шасси
        if (isFieldCentric) {
            double allianceOffset = isBlueAlliance ? Math.PI : 0;
            follower.setTeleOpDrive(forward, strafe, turn, false, allianceOffset);
        } else {
            // Robot-Centric
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }
    }

    // ==========================================
    // ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
    // ==========================================

    public Follower getFollower() { return follower; }

    public void setSnapTarget(double angleRadians) { snapTargetAngle = angleRadians; }

    public void toggleFieldCentric() { isFieldCentric = !isFieldCentric; }

    public void holdCurrentPosition() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getPose().getHeading());
    }

    public void setPose(Pose pose) { follower.setPose(pose); }

    public Pose getPose() { return follower.getPose(); }
}