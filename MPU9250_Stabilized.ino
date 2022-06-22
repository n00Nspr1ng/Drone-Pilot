void MPU9250_Stabilized()
{
  while(1)
  {
    for(int i = 0; i < 10; ++i)
    {
      get_Angle_MPU_9250();
      delay(100);
      ypr0_sum += abs(ypr[0] - ypr0_prev);
        Serial.print("roll = ");
        Serial.print(ypr[2]);
        Serial.print("\tpitch = ");
        Serial.print(ypr[1]);
        Serial.print("\tyaw = ");
        Serial.print(ypr[0]);
        Serial.print("\tsum = ");
        Serial.println(ypr0_sum* 180/M_PI);
      ypr0_prev = ypr[0];
    }

    if(ypr0_sum * 180/M_PI < 1.00)
    {
      ref_roll = ypr[2];
      ref_pitch = ypr[1];
      break;
    }
    else
    {
      ypr0_sum = 0;
    }
  }
}
