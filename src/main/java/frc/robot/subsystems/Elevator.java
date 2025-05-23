// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorLeader = new TalonFX(10);
  private TalonFX elevatorFollower = new TalonFX(9);
  private Follower follower = new Follower(10, false);
  private boolean safeUp;
  private boolean safeDown;
  private DigitalInput upperLimit = new DigitalInput(2);
  private DigitalInput lowerLimit = new DigitalInput(3);
  private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(0, 360, 0);
  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    elevatorFollower.setControl(follower);
  }
  
  public void evaluateSafe(){
    if(upperLimit.get()){
      safeUp = false;
    } else {
      safeUp = true;
    }
    if(lowerLimit.get()){
      safeDown = false;
    } else {
      safeDown = true;
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command elevatorRunCommand(double leftStickYValue) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          evaluateSafe();
          if(leftStickYValue < 0 && safeUp){
            elevatorLeader.set(leftStickYValue);
          } else if (leftStickYValue > 0 && safeDown){
            elevatorLeader.set(leftStickYValue);
          } else {
            elevatorLeader.set(0);
          }

        });
  }

  public void setPosition(double targetPosition){
    
  }

  



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
