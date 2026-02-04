package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Util.Dashboard.HardwareFaultTracker;

public class Shooter implements Subsystem {

    //Hardware

    private final TalonFX shooterMotor =
        new TalonFX(HardwareConstants.SHOOTER_CAN);

    // Control

    // Phoenix expects rotations per second
    private final VelocityVoltage rpmRequest = new VelocityVoltage(0.0);

    // Alerts

    private final Alert shooterAlert =
        new Alert("Shooter Motor Error", AlertType.kError);

    // Construction

    public Shooter() {

        TalonFXConfiguration config = new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.12)
                    .withKI(0.0)
                    .withKD(0.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0));

        shooterMotor.getConfigurator().apply(config);
    }

    // Shooter Control

    // Shootr speed in RPM
    public void setRPM(double rpm) {
        shooterMotor.setControl(
            rpmRequest.withVelocity(rpm / 60.0)
        );
    }

    //Stop the shooting
    public void stop() {
        shooterMotor.set(0.0);
    }

    //return current shooter speed in RPM
    public double getRPM() {
        return shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    // return true if shooter is within tolerance of target RPM
    public boolean atRPM(double targetRPM, double toleranceRPM) {
        return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
    }

    // checkHardware

    public void checkHardware() {
        HardwareFaultTracker.checkFault(
            shooterAlert,
            shooterMotor.hasActiveFault() || shooterMotor.hasActiveWarning());
    }
}
