// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.LaserCan;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import au.grapplerobotics.CanBridge;

import static com.revrobotics.spark.SparkLowLevel.*;
import static edu.wpi.first.units.Units.Seconds;

public class Robot extends TimedRobot {

    private final MutTime lastTime = Seconds.mutable(0.0);
    private final MutTime currentTime = Seconds.mutable(0.0);

    private final double JKgMetersSquared = 1;
    private final SparkMax motor0 = new SparkMax(16, MotorType.kBrushless);
    private final SparkMax motor1 = new SparkMax(18, MotorType.kBrushless);
    private final XboxController controller = new XboxController(0);

    private final Solenoid solenoid1 =  new Solenoid(PneumaticsModuleType.CTREPCM, 4);
    private final Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 5);



    private final SparkMaxSim motor0Sim = new SparkMaxSim(motor0, DCMotor.getNeo550(1));
    private final SparkMaxSim motor1Sim = new SparkMaxSim(motor1, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant0 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim0 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant1 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim1 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final LinearSystem<N2, N1, N2> plant2 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), JKgMetersSquared, 12.0);
    private final DCMotorSim dcMotorSim2 = new DCMotorSim(plant0, DCMotor.getNeo550(1));
    private final double deadband = 0.05;

    private final LaserCan laser1 = new LaserCan(9);


    public Robot() {
        // CanBridge.runTCP();
        ClosedLoopConfig closedLoopConfig = new SparkMaxConfig()
                .closedLoop
                .p(0.0000000001).i(0.0).d(0.0).velocityFF(1.0/11000.0);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
         sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        sparkMaxConfig.apply(closedLoopConfig);
        motor1.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        motor0.configure(sparkMaxConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        lastTime.mut_replace(Timer.getFPGATimestamp(), Seconds);
    }

    @Override
    public void teleopPeriodic() {
        double leftValue = MathUtil.applyDeadband(-controller.getLeftY(), deadband);
        double rightValue = MathUtil.applyDeadband(-controller.getRightY(), deadband);
        double leftTrigger = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband);
        double rightTrigger = -MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
        double triggerNet = leftTrigger + rightTrigger;

        double triggerVoltage = 12.0 * triggerNet;

        // motor0.setVoltage(leftValue * 12.0);
        // motor1.setVoltage(rightValue * 12.0);
        // motor2.setVoltage(triggerVoltage);

        // motor0.getClosedLoopController().setReference(leftValue * 11000, SparkBase.ControlType.kVelocity);
        // if(laser1.getMeasurement().distance_mm > 10.0){
        //     motor1.getClosedLoopController().setReference(rightValue * 11000, SparkBase.ControlType.kVelocity);
        // }else{
        //     motor1.stopMotor();
        // }


        if(controller.getAButtonPressed()){
            solenoid1.toggle();
        }

        if(controller.getBButtonPressed()){
            solenoid2.toggle();
        }

        var measure = laser1.getMeasurement();
        if(measure != null){
            System.out.println(measure.distance_mm);
        }else{
            System.out.println("I'm null for some reason");
        }

        // SmartDashboard.putNumber("LASERCAN", laser1.getMeasurement().distance_mm);
        SmartDashboard.putNumber("Motor 0 Current", motor0.getOutputCurrent());
        SmartDashboard.putNumber("Motor 0 Velocity RPM", motor0.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        currentTime.mut_replace(Timer.getFPGATimestamp(), Seconds);
        double deltaTSeconds = currentTime.baseUnitMagnitude() - lastTime.baseUnitMagnitude();
        lastTime.mut_replace(currentTime);
        dcMotorSim0.setInputVoltage(motor0.getAppliedOutput() * motor0.getBusVoltage());
        dcMotorSim0.update(deltaTSeconds);
        motor0Sim.iterate(dcMotorSim0.getAngularVelocityRPM(), 12.0, deltaTSeconds);



    }
}
