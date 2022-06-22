void MotorInit()
{
  while(1)
  {
    channel_1 = map(receiver_input_channel_1, fromLow, fromHigh, toLow, toHigh);
    channel_2 = map(receiver_input_channel_2, fromLow, fromHigh, toLow, toHigh);
    channel_3 = map(receiver_input_channel_3, fromLow, fromHigh, toLow, toHigh);
    channel_4 = map(receiver_input_channel_4, fromLow, fromHigh, toLow, toHigh);

    cw1.writeMicroseconds(channel_3);
    ccw1.writeMicroseconds(channel_3);
    cw2.writeMicroseconds(channel_3);
    ccw2.writeMicroseconds(channel_3);

    if (channel_1 >= 2000 & channel_2 >= 2000){
      delay(4000);
      break;
    }
  }
}
