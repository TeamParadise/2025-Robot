
package com.team1165.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import com.team1165.robot.subsystems.elevator.Elevator;


public  class ElevatorPosition extends Command
{
    private final Elevator elevator;

 public ElevatorPosition(Elevator elevator)
    {
          this.elevator = elevator;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevator);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        
    }
    
    @Override
    public boolean isFinished()
    {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        
    }
}
