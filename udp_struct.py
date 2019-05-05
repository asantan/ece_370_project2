from ctypes import Structure, c_double, c_uint16

class message(Structure):
	_pack_=1
	_fields_ = [	("velocity", c_double), 
			("theta", c_double), 
			("reset", c_uint16)]

