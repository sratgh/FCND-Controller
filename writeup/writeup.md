# FCND Controls CPP Writeup

![3D Control Architecture](writeup/3D_Control_Architecture.png)

## Testing it Out ###
When testing the drone out, to make it stay in the air while adjusting the weight in the config file I found the following weight to work:
```
Mass = 0.23
```


## Body rate and roll/pitch control (scenario 2) ###

1. Implement body rate control

For this task I implemented the following C++ and could see the drone stable in the air, flying off up quite quickly, which according to the readme is fine since the angle is not yet being controlled back to zero.

`BodyRateControl()`

```
// 1. Calculate errors of the three bodyrates p, q and r
V3F rateError = pqrCmd - pqr;

// 2. Pack moments of inertia into vector for simple multiplication
V3F momentOfInertia = V3F(Ixx, Iyy, Izz);

// 2. Calculate P-Term command and multiply with moment of inertia to get the actual torque / moment command
momentCmd = momentOfInertia * kpPQR * rateError;
```

`GenerateMotorCommands`

```
// 1. Calculate forces from moments
float xForce = momentCmd.x / (L / sqrt(2));
float yForce = momentCmd.y / (L / sqrt(2));
float zForce = momentCmd.z / kappa;

// 2. Calculate desired thrust
float frontLeft = (collThrustCmd + zForce + xForce + yForce) / 4.f; // front left
float frontRight = (collThrustCmd - zForce - xForce + yForce) / 4.f; // front right
float rearLeft = (collThrustCmd - zForce + xForce - yForce) / 4.f; // rear left
float rearRight = (collThrustCmd + zForce - xForce - yForce) / 4.f; // rear right

cmd.desiredThrustsN[0] = CONSTRAIN(frontLeft, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[1] = CONSTRAIN(frontRight, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[2] = CONSTRAIN(rearLeft, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[3] = CONSTRAIN(rearRight, minMotorThrust, maxMotorThrust);
```

2. Implement roll / pitch control

I implemented the function `RollPitchControl` as follows:

```
// 1. Extract necessary terms from rotational matrix
float R13 = R(0,2);
float R23 = R(1,2);
float R33 = R(2,2);
float R21 = R(1,0);
float R22 = R(1,1);
float R12 = R(0,1);
float R11 = R(0,0);

if (collThrustCmd > 0)
{
    // 2. Convert force to acceleration by dividing it by the mass of the drone
    float clippedAccelX = CONSTRAIN(accelCmd[0] * mass / collThrustCmd, -maxTiltAngle, maxTiltAngle);
    float clippedAccelY = CONSTRAIN(accelCmd[1] * mass / collThrustCmd, -maxTiltAngle, maxTiltAngle);

    // 3. Calculate intermediate x, y acceleration commands
    float xTemp = kpBank * (clippedAccelX - R13);
    float yTemp = kpBank * (clippedAccelY - R23);

    // 4. Calculate roll and pitch commands with terms from rotational matrix
    float pCmd = (1 / R33) * (R21 * xTemp - R11 * yTemp);
    float qCmd = (1 / R33) * (R22 * xTemp - R12 * yTemp);

    // 5. Assign roll and pitch commands to return variable
    pqrCmd.x = pCmd;
    pqrCmd.y = qCmd;

}
else
{
    pqrCmd.x = 0.0;
    pqrCmd.y = 0.0;
}
```
This gave me the expected behavior on the simulator (still with the drone flying off though).

## Position/velocity and yaw angle control (scenario 3) ##

I implemented the functions `LateralPositionControl()` and `AltitudeControl()` as follows:

`LateralPositionControl()`

```
// 1. Limit input velocity and acceleration
accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

// 2. Calculate acceleration command by adding P-Term for position and velocity and add acceleration command
accelCmd = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmd;

// 3. Limit output acceleration command
accelCmd.x = - CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = - CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```

`AltitudeControl()`

```
// 1. Calculate Z-Position error
float posError = (posZCmd - posZ);

// 2. Calculate Z-Position P-Term
float pTermPos = kpPosZ * posError;

// 3. Calculate Z-Position I-Term
integratedAltitudeError += KiPosZ * posError * dt;

// 4. Calculate Z-Velocity error
float velError = velZCmd - velZ;

// 5. Calculate Z-Velocity P-Term
float pTermVel = kpVelZ * velError;

// 6. Sum up Z-Position, Z-Velocity and Z-Acceleration terms/commands to get the total Z-Acceleration
float termSum = pTermPos + pTermVel + integratedAltitudeError + accelZCmd;

// 7. Subtract gravity from total Z-Acceleration and only take the Pitch and Roll cosines to get the pure Z-Component
float zAccel = (termSum - 9.81f ) / R(2,2);


//if (zAccel > 0)
//zAccel = 0;

// 8. Calculate thrust
thrust = -mass * zAccel;
```


## Non-idealities and robustness (scenario 4) ##

In this part I played with the parameters in the config a bit more to achieve the desired behavior.

## Tracking trajectories ##

The scenario for tracking the trajectories worked out of the box, when trying the first time.
