package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drive {
    private Follower follower;
    private boolean isFieldCentric = true;
    private boolean isHoldingAngle = false;

    // Множитель скорости (для медленного режима)
    public double speedMultiplier = 1.0;

    // ПИД для удержания угла прицеливания (настрой эти значения!)
    public static double kP = 1.0, kI = 0.0, kD = 0.1;
    public double targetAngle = 0;
    private PIDController headingController;

    public void init(HardwareMap hw) {
        // Используем твой единый файл констант для создания шасси
        follower = Constants.createFollower(hw);

        headingController = new PIDController(kP, kI, kD);
    }

    // Вызывается в onStart() ТелеОпа
    public void startTeleop() {
        follower.startTeleopDrive();
    }

    public void periodic() {
        // Обязательный апдейт Pedro Pathing (он сам опрашивает Pinpoint)
        follower.update();
    }

    // ==========================================
    // ЛОГИКА УПРАВЛЕНИЯ (Вызывается из TeleOp)
    // ==========================================

    public void drive(GamepadEx gamepad, boolean isBlueAlliance) {
        // 1. Считываем стики и применяем кубическую кривую для плавности (замена InputScaler)
        double forward = Math.pow(gamepad.getLeftY(), 3) * speedMultiplier;
        double strafe = Math.pow(-gamepad.getLeftX(), 3) * speedMultiplier; // Инвертируем X
        double turn = Math.pow(-gamepad.getRightX(), 3) * speedMultiplier;

        // 2. Логика удержания угла (Авто-прицеливание)
        if (isHoldingAngle) {
            headingController.setPID(kP, kI, kD);
            // Считаем разницу между текущим углом и нужным
            double currentHeading = follower.getPose().getHeading();

            // Заменяем ручной поворот со стика на команду от ПИД-регулятора
            turn = headingController.calculate(currentHeading, targetAngle);
        }

        // 3. Передаем мощности в Pedro Pathing
        if (isFieldCentric) {
            // Если мы за синих, поле развернуто на 180 градусов (PI радиан)
            double allianceOffset = isBlueAlliance ? Math.PI : 0;
            follower.setTeleOpDrive(forward, strafe, turn, false, allianceOffset);
        } else {
            // Робото-центричный режим (как машинка на пульте управления)
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }
    }

    // ==========================================
    // ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
    // ==========================================

    public void faceTarget(Pose targetPose) {
        Pose robotPose = follower.getPose();
        // Считаем угол от робота до цели с помощью арктангенса
        targetAngle = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        isHoldingAngle = true;
    }

    public void stopFacing() {
        isHoldingAngle = false;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void holdCurrentPosition() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getPose().getHeading());
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.getPose();
    }
}