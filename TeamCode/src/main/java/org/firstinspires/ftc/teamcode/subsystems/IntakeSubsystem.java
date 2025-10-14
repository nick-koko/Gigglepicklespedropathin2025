/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * NextFTC Subsystem for controlling the intake mechanism.
 * This subsystem manages a single motor for intake/outtake operations.
 */
public class IntakeSubsystem implements Subsystem {

    private DcMotor motor;
    
    // Configurable power levels
    public static double INTAKE_POWER = 0.7;
    public static double OUTTAKE_POWER = -0.7;

    /**
     * Initialize the subsystem with the hardware map.
     * Call this method from your OpMode's onInit().
     */
    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set the intake motor to intake (positive power).
     */
    public void intake() {
        motor.setPower(INTAKE_POWER);
    }

    /**
     * Set the intake motor to outtake (negative power).
     */
    public void outtake() {
        motor.setPower(OUTTAKE_POWER);
    }

    /**
     * Stop the intake motor.
     */
    public void stop() {
        motor.setPower(0.0);
    }

    /**
     * Set a custom power level for the intake motor.
     * @param power Power level between -1.0 and 1.0
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Get the current power of the intake motor.
     * @return Current motor power
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Periodic method called automatically by NextFTC.
     * Use this for telemetry updates or state monitoring.
     */
    @Override
    public void periodic() {
        // Optional: Add telemetry or state updates here
    }
}

