* When changing how odometry is calculated, I had to tune the frequency of how often MotorTimerCallback was called in the timer.
  when called 20 times per second, here are the teensy_stats output:

  ```code
  data: '{"Stats": {"loops":116,"Ms":1007.7,"mdls":[{"n":"Batt","MnMxAv":[0.0,0.6,0.0]},{"n":"uRos","MnMxAv":[0.0,19.2,1.9]},{"n":"Rlay","MnMxAv":[0.0,0.0,0.0]},{"n":"Robo","MnMxAv":[5.9,7.2,6.7]},{"n":"TSd","MnMxAv":[0.0,0.0,0.0]}]}}'
  ```

  When I changed the frequency to 40 times per second, here are the new state:

  ```code
  data: '{"Stats": {"loops":77,"Ms":1004.7,"mdls":[{"n":"Batt","MnMxAv":[0.0,0.5,0.0]},{"n":"uRos","MnMxAv":[0.0,40.4,6.2]},{"n":"Rlay","MnMxAv":[0.0,0.0,0.0]},{"n":"Robo","MnMxAv":[6.2,7.3,6.8]},{"n":"TSd","MnMxAv":[0.0,0.0,0.0]}]}}'
  ```

  It's important to make sure that the loop count is at least as high as the frequency of the timer.