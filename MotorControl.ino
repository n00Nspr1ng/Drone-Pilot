void MotorControl()
{
  divider = 1.0;
  cw1_out = throttleOutput + constrain(- rollRateOutput/divider + pitchRateOutput/divider - yawRateOutput/divider, -500, 500);
  ccw1_out = throttleOutput + constrain(rollRateOutput/divider + pitchRateOutput/divider + yawRateOutput/divider, -500, 500);
  cw2_out = throttleOutput + constrain(rollRateOutput/divider - pitchRateOutput/divider - yawRateOutput/divider, -500, 500);
  ccw2_out = throttleOutput + constrain(- rollRateOutput/divider - pitchRateOutput/divider + yawRateOutput/divider, -500, 500);
  
  cw1.writeMicroseconds(constrain(cw1_out + 1050, 1050, 2154) + 35);
  ccw1.writeMicroseconds(constrain(ccw1_out + 1050, 1050, 2159) + 65);
  cw2.writeMicroseconds(constrain(cw2_out + 1050, 1050, 2152));
  ccw2.writeMicroseconds(constrain(ccw2_out + 1050, 1050, 2152));
}
