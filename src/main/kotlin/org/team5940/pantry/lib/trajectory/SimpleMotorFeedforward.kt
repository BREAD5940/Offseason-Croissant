///*----------------------------------------------------------------------------*/
///* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package org.team5940.pantry.lib.trajectory
//
///**
// * A helper class that computes feedforward outputs for a simple permanent-magnet DC motor.
// */
//class SimpleMotorFeedforward
///**
// * Creates a new SimpleMotorFeedforward with the specified gains.  Units of the gain values
// * will dictate units of the computed feedforward.
// *
// * @param ks The static gain.
// * @param kv The velocity gain.
// * @param ka The acceleration gain.
// */
//(val ks: Double, val kv: Double, val ka: Double = 0.0) {
//
//    /**
//     * Calculates the feedforward from the gains and setpoints.
//     *
//     * @param velocity     The velocity setpoint.
//     * @param acceleration The acceleration setpoint.
//     * @return The computed feedforward.
//     */
//    fun calculate(velocity: Double, acceleration: Double = 0.0): Double {
//        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration
//    }
//}
