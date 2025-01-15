// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.revrobotics.spark.SparkLowLevel.*;

public class Robot extends TimedRobot {

    private final double JKgMetersSquared = 0.01;
    private final SparkMax motor0 = new SparkMax(16, MotorType.kBrushless);
    private final SparkMax motor1 = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax motor2 = new SparkMax(24, MotorType.kBrushless);
    private final CommandXboxController controller = new CommandXboxController(0);
    private final SparkMaxSim motor0Sim = new SparkMaxSim(motor0, DCMotor.getNeo550(1));
    private final SparkMaxSim motor1Sim = new SparkMaxSim(motor1, DCMotor.getNeo550(1));
    private final SparkMaxSim motor2Sim = new SparkMaxSim(motor2, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant0 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim0 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant1 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim1 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant2 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim2 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final double deadband = 0.05;


    public Robot() {
        ClosedLoopConfig closedLoopConfig = new SparkMaxConfig()
                .closedLoop
                .p(0.1).i(0.0).d(0.0).velocityFF(1.0/11000.0);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(closedLoopConfig);
        motor2.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motor1.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motor0.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void teleopPeriodic() {
        double leftValue = MathUtil.applyDeadband(-controller.getLeftY(), deadband);
        double rightValue = MathUtil.applyDeadband(-controller.getRightY(), deadband);
        double leftTrigger = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband);
        double rightTrigger = -MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
        double triggerNet = leftTrigger + rightTrigger;

        double triggerVoltage = 12.0 * triggerNet;

        motor0.setVoltage(leftValue * 12.0);
        motor1.setVoltage(rightValue * 12.0);
        motor2.setVoltage(triggerVoltage);

        // motor0.getClosedLoopController().setReference(leftValue * 11000, SparkBase.ControlType.kVelocity);
        // motor1.getClosedLoopController().setReference(rightValue * 11000, SparkBase.ControlType.kVelocity);
        // motor2.getClosedLoopController().setReference(leftValue * 11000, SparkBase.ControlType.kVelocity);


        SmartDashboard.putNumber("Motor 2 Current", motor2.getOutputCurrent());
        SmartDashboard.putNumber("Motor 2 Velocity RPM", motor2.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        motor0Sim.setAppliedOutput(motor0Sim.getSetpoint());
        motor0Sim.setBusVoltage(12.0);
        motor0Sim.setVelocity(dcMotorSim0.getAngularVelocityRPM());
        motor0Sim.setPosition(dcMotorSim0.getAngularPositionRotations());
        dcMotorSim0.setInputVoltage(motor0Sim.getSetpoint());
        dcMotorSim0.update(0.020);

        motor1Sim.setAppliedOutput(motor1Sim.getSetpoint());
        motor1Sim.setBusVoltage(12.0);
        motor1Sim.setVelocity(dcMotorSim0.getAngularVelocityRPM());
        motor1Sim.setPosition(dcMotorSim0.getAngularPositionRotations());
        dcMotorSim1.setInputVoltage(motor1Sim.getSetpoint());
        dcMotorSim1.update(0.020);

        motor2Sim.setAppliedOutput(motor2Sim.getSetpoint());
        motor2Sim.setBusVoltage(12.0);
        motor2Sim.setVelocity(dcMotorSim2.getAngularVelocityRPM());
        motor2Sim.setPosition(dcMotorSim0.getAngularPositionRotations());
        dcMotorSim2.setInputVoltage(motor2Sim.getSetpoint());
        dcMotorSim2.update(0.020);


    }
}
