from ctypes import *

class message(Structure):
	_pack_=1
	_fields_ = [	("velocity", c_double), 
			("theta", c_double), 
			("rst", c_uint8)]

