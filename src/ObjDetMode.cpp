#include "sciroc_objdet/ObjDetMode.h"

//ObjDetMode::ObjDetMode() 						{ mode_ = Mode::NONE; 			}

void ObjDetMode::setModeNone()      { mode_ = Mode::NONE;       }
void ObjDetMode::setModeEnumerate() { mode_ = Mode::ENUMERATE;  }
void ObjDetMode::setModeClassify()  { mode_ = Mode::CLASSIFY;   }
void setModeCompare()               { mode_ = Mode::COMPARE;    }

bool isModeNone() 					{ return mode_ == Mode::NONE; 			}
bool isModeEnumerate()			{ return mode_ == Mode::ENUMERATE; 	}
bool isModeClassify()				{ return mode_ == Mode::CLASSIFY; 	}
bool isModeCompare()				{ return mode_ == Mode::COMPARE; 		}