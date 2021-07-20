#include "sciroc_objdet/ObjDetMode.h"

void ObjDetMode::setModeNone()      { mode_ = Mode::NONE;       }
void ObjDetMode::setModeEnumerate() { mode_ = Mode::ENUMERATE;  }
void ObjDetMode::setModeClassify()  { mode_ = Mode::CLASSIFY;   }
void ObjDetMode::setModeCompare()               { mode_ = Mode::COMPARE;    }

bool ObjDetMode::isModeNone() 					{ return mode_ == Mode::NONE; 			}
bool ObjDetMode::isModeEnumerate()			{ return mode_ == Mode::ENUMERATE; 	}
bool ObjDetMode::isModeClassify()				{ return mode_ == Mode::CLASSIFY; 	}
bool ObjDetMode::isModeCompare()				{ return mode_ == Mode::COMPARE; 		}