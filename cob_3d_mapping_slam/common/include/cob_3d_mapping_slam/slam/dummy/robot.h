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
      return 0.5f;
    }

    static float getMaxSpeedRotationPerSecond()
    {
      return 0.4f;
    }

  };
}



#endif /* ROBOT_H_ */
