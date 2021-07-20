#ifndef MODE_H
#define MODE_H

#include <ros/ros.h>

class ObjDetMode
{
  public:
		ObjDetMode() : mode_(Mode::NONE){}

    void setModeNone();
    void setModeEnumerate();
    void setModeClassify();
    void setModeCompare();

    bool isModeNone();
    bool isModeEnumerate();
    bool isModeClassify();
    bool isModeCompare();

  protected:

    enum class Mode
    {
      NONE = -1,
      ENUMERATE,
      CLASSIFY,
      COMPARE
    };
    Mode mode_;
};

#endif