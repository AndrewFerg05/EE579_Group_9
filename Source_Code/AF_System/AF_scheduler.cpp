#include "AF_scheduler.h"

Time increment(Time newtime)
{
  newtime.ms++;
  if(newtime.ms==1000)
  {
    newtime.s++;
    newtime.ms = 0;
  }
  if(newtime.s==1000) newtime.s = 0;
  return newtime;
}


Time schedule (int duration)
{
  Time newtime;
  newtime.s = CurrentTime.s;
  newtime.ms = CurrentTime.ms + duration;  //Newtime = current time + event schedule duration

  while (newtime.ms > 999)
  {
      newtime.s++;
      newtime.ms -= 1000;                     //Convert 1000ms into 1s
  }

  if(newtime.s > 1000)
  {
      newtime.s -= 1000;                    //Loop cycles at 1000s
  }

  return newtime;                               //Return Scheduled Time
}