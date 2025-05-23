package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public TalonFX armLeader = new TalonFX(12);
    public TalonFX armFollower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    //Constructor
    public Arm(){
        armFollower.setControl(follower);
    }

    public Command armRunCommand(double rightStickXValue){
        return runOnce(
        () -> {
            armLeader.set(rightStickXValue);
        });

        
    }
}
