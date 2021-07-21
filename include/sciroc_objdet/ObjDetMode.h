#ifndef MODE_H
#define MODE_H

#include <ros/ros.h>

class ObjDetMode
{
  public:
		ObjDetMode() : mode_(Mode::NONE){}
    ObjDetMode(int mode) 
      : mode_((mode <= static_cast<int>(Mode::NONE) && mode >= 0) ? 
        static_cast <Mode>(mode) :  Mode::NONE)
      {}

    void setModeNone();
    void setModeEnumerate();
    void setModeClassify();
    void setModeCompare();

    bool isModeNone();
    bool isModeEnumerate();
    bool isModeClassify();
    bool isModeCompare();

    enum class Mode
    {
      ENUMERATE,
      CLASSIFY,
      COMPARE,
      NONE
    };

  protected:

    Mode mode_;
    /*
    Mode boundInt(int mode){
      if (mode > static_cast<int>(Mode::NONE) || mode < 0)
        return Mode::NONE;
      return static_cast<Mode>(mode);
    }*/
};

std::ostream &operator<<(std::ostream &os, ObjDetMode o);

#endif