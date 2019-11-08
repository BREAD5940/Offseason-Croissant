// package org.team5940.pantry.lib
//
// import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
// import org.ghrobotics.lib.mathematics.units.SIUnit
// import org.ghrobotics.lib.mathematics.units.derived.Acceleration
// import org.ghrobotics.lib.mathematics.units.derived.Velocity
// import org.ghrobotics.lib.mathematics.units.second
// import org.ghrobotics.lib.motors.FalconMotor
// import org.ghrobotics.lib.motors.ctre.FalconSRX
// import org.ghrobotics.lib.motors.rev.FalconMAX
// import org.ghrobotics.lib.simulation.SimFalconMotor
//
// fun configureMotor(
//    motor: FalconMotor<*>,
//    cruiseVel: Velocity<*>,
//    cruiseAccel: Acceleration<*>,
//    pidfSlot: Int,
//    currentLimitConfig: FalconSRX.CurrentLimitConfig,
//    slotConfiguration: SlotConfiguration,
//    minPosition: SIUnit<*>,
//    maxPosition: SIUnit<*>
// ):
//        FalconMotor<*> {
//
//
//
//    motor.run {
//
//        motionProfileCruiseVelocity = cruiseVel
//        motionProfileCruiseVelocity = cruiseAccel
//        useMotionProfileForPosition = true
//        brakeMode = true
//
//        when (motor) {
//
//            is FalconSRX -> {
//                motor.configCurrentLimit(
//                        true,
//                        currentLimitConfig
//                )
//
//                val miniMotor = motor.motorController
//
//                miniMotor.configPeakOutputForward(1.0, 0)
//                miniMotor.configPeakOutputReverse(-1.0, 0)
//                miniMotor.config_kP(pidfSlot, slotConfiguration.kP, 0)
//                miniMotor.config_kI(pidfSlot, slotConfiguration.kI, 0)
//                miniMotor.config_kD(pidfSlot, slotConfiguration.kD, 0)
//                miniMotor.config_kF(pidfSlot, slotConfiguration.kF, 0)
//                miniMotor.config_kF(pidfSlot, slotConfiguration.kF, 0)
//                miniMotor.config_IntegralZone(pidfSlot, slotConfiguration.integralZone, 0)
//                miniMotor.configMaxIntegralAccumulator(pidfSlot, slotConfiguration.maxIntegralAccumulator, 0)
//                miniMotor.configAllowableClosedloopError(pidfSlot, slotConfiguration.allowableClosedloopError, 0)
//                miniMotor.configClosedLoopPeakOutput(pidfSlot, slotConfiguration.closedLoopPeakOutput, 0)
//                miniMotor.configClosedLoopPeriod(pidfSlot, slotConfiguration.closedLoopPeriod, 0)
//
// //                    miniMotor.configForwardLimitSwitchSource(maxLimitConfig.source, maxLimitConfig.limitNormal, miniMotor.deviceID, 0)
// //                    miniMotor.configReverseLimitSwitchSource(minLimitConfig.source, minLimitConfig.limitNormal, miniMotor.deviceID, 0)
//
//                miniMotor.configForwardSoftLimitThreshold(motor.model.fromNativeUnitPosition(maxPosition.value).toInt(), 0)
//                miniMotor.configReverseSoftLimitThreshold(motor.model.fromNativeUnitPosition(minPosition.value).toInt(), 0)
//
//                miniMotor.configForwardSoftLimitEnable(true, 0)
//                miniMotor.configReverseSoftLimitEnable(true, 0)
//            }
//            is FalconMAX -> {
//                val motorController = motor.canSparkMax
//                val pid = motorController.pidController
//
//                pid.setOutputRange(-1.0, 1.0)
//                pid.setP(slotConfiguration.kP, pidfSlot)
//                pid.setI(slotConfiguration.kI, pidfSlot)
//                pid.setD(slotConfiguration.kD, pidfSlot)
//                pid.setFF(slotConfiguration.kF, pidfSlot)
//                pid.setIZone(slotConfiguration.integralZone.toDouble(), pidfSlot)
//                pid.setIMaxAccum(slotConfiguration.maxIntegralAccumulator, pidfSlot)
//                pid.setSmartMotionAllowedClosedLoopError(slotConfiguration.allowableClosedloopError.toDouble(), pidfSlot)
//
// //                    val maxPolarity = if (maxLimitConfig.limitNormal == LimitSwitchNormal.NormallyOpen) CANDigitalInput.LimitSwitchPolarity.kNormallyClosed else CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
// //                    val minPolarity = if (minLimitConfig.limitNormal == LimitSwitchNormal.NormallyOpen) CANDigitalInput.LimitSwitchPolarity.kNormallyClosed else CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
//
// //                    motor.getForwardLimitSwitch(maxPolarity)
// //                    motor.getReverseLimitSwitch(minPolarity)
//            }
//            is SimFalconMotor -> {
//            }
//            else -> TODO("Config not implemented for non-CTR and non-REV joint!")
//        }
//
//        return motor
//    }
// }
