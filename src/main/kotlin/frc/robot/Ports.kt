package frc.robot

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.InvertType
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits

object Ports {

    const val kPCMID = 8

    object DrivePorts {
        val LEFT_PORTS = listOf(1, 2)
        val RIGHT_PORTS = listOf(3, 4)
        val SHIFTER_PORTS = listOf(4, 5)
    }

    object IntakePorts {
        const val CARGO_PORT = 51
        const val HATCH_PORT = 50
        val PISTON_PORTS = listOf(0, 1)
    }

    object SuperStructurePorts {
        object ElevatorPorts {
            val TALON_PORTS = listOf(21, 22, 23, 24)
            val MASTER_INVERTED = false
            val MASTER_SENSOR_PHASE = true
            val FOLLOWER_INVERSION = listOf(InvertType.OpposeMaster, InvertType.FollowMaster, InvertType.FollowMaster)
            val LENGTH_MODEL = NativeUnitLengthModel(4096.nativeUnits, 1.5.inch / 2)
            val SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative
        }
        object ProximalPorts {
            val TALON_PORTS = listOf(31, 32)
            val TALON_INVERTED = true
            val TALON_SENSOR_PHASE = true
            val FOLLOWER_INVERSION = listOf(InvertType.OpposeMaster)
            val ROTATION_MODEL = NativeUnitRotationModel(4096.nativeUnits * 28 / 3)
            val SENSOR = FeedbackDevice.RemoteSensor0
        }
        object WristPorts {
            val TALON_PORTS = 33
            val TALON_INVERTED = true
            val TALON_SENSOR_PHASE = false
            val ROTATION_MODEL = NativeUnitRotationModel(4096.nativeUnits * 8)
            val SENSOR = FeedbackDevice.RemoteSensor0
        }
    }
}
