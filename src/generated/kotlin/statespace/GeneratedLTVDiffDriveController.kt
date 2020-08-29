package statespace

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import org.team5940.pantry.lib.statespace.LTVDiffDriveController

val generatedLTVDiffDriveControllerHighGear get() = LTVDiffDriveController(
    28.518621431415458, -17.368623458959274, 5.839481750009456, 1.8866960096663798,
        -17.067825897055744, -10.35374903626718, 7.774759626622991,
        DifferentialDriveKinematics(0.3302))
