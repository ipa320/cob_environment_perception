/*
 * robot.h
 *
 *  Created on: 31.05.2012
 *      Author: josh
 */

#ifndef ROBOT_H_
#define ROBOT_H_


namespace Dummy
{

  class RobotParameters
  {
  public:

    static float getMaxSpeedTranslationPerSecond()
    {
      return 0.7f;
    }

    static float getMaxSpeedRotationPerSecond()
    {
      return 0.5f;
    }

  };

  class RobotParametersSlow
  {
  public:

    static float getMaxSpeedTranslationPerSecond()
    {
      return 0.3f;
    }

    static float getMaxSpeedRotationPerSecond()
    {
      return 0.25f;
    }

  };

}



#endif /* ROBOT_H_ */
