#include "sciroc_objdet/ObjDetMode.h"

void ObjDetMode::setMode(int mode)
{
	mode_ = (mode <= static_cast<int>(Mode::NONE) && mode >= 0) ? static_cast<Mode>(mode) : Mode::NONE;
}

void ObjDetMode::setModeNone()      { mode_ = Mode::NONE;       }
void ObjDetMode::setModeEnumerate() { mode_ = Mode::ENUMERATE;  }
void ObjDetMode::setModeClassify()  { mode_ = Mode::CLASSIFY;   }
void ObjDetMode::setModeCompare()               { mode_ = Mode::COMPARE;    }

bool ObjDetMode::isModeNone() 					{ return mode_ == Mode::NONE; 			}
bool ObjDetMode::isModeEnumerate()			{ return mode_ == Mode::ENUMERATE; 	}
bool ObjDetMode::isModeClassify()				{ return mode_ == Mode::CLASSIFY; 	}
bool ObjDetMode::isModeCompare()				{ return mode_ == Mode::COMPARE; 		}
int ObjDetMode::getModeInt() 						{ return static_cast<int>(mode_); 	}

std::ostream& operator<<(std::ostream& os, ObjDetMode o)
{
	if (o.isModeEnumerate())	os << "ENUMERATE";
	if (o.isModeClassify())		os << "CLASSIFY";
	if (o.isModeCompare())		os << "COMPARE";
	if (o.isModeNone())				os << "NONE";
	else
			os.setstate(std::ios_base::failbit);
	return os;
}
