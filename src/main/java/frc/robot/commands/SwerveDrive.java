//packages commands
package frc.robot.commands;

//imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

//creates class under commands
public class SwerveDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    //defines swerve drive, inputs, and field oriented
    private final SwerveSubsystem m_swerveDrive;
    private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
    private final boolean m_isFieldRelative;

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDriveSubsystem The subsystem used by this command.
     */
    //defines what goes into a swerve drive
    public SwerveDrive(SwerveSubsystem swerveDriveSubsystem, DoubleSupplier throttleInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, boolean isFieldRelative) {
        //makes variables we can use elsewhere
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;
        m_isFieldRelative = isFieldRelative;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //accounts for potential joystick drift
        double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.15 ? m_throttleInput.getAsDouble() : 0;
        double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.15 ? m_strafeInput.getAsDouble() : 0;
        double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.15 ? m_rotationInput.getAsDouble() : 0;
        // Forward/Back Throttle, Left/Right Strafe, Left/Right Turn

        //actual drive method, takes in throttle, strafe, rotation, if it's field oriented, and if it's an open loop
        m_swerveDrive.drive(throttle * .8, strafe * .8, rotation * .8, m_isFieldRelative, true);
    }
}
