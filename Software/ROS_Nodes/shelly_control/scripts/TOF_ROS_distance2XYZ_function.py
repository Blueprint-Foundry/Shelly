
##	  4x4 SPAD quads are numbered left to right, top to bottom:
##	  01 02 03 04 
##	  05 06 07 08
##	  09 10 11 12
##	  13 14 15 16
##        spad centers
##	  145  117 209 241 
##	  149  181 213 245
##	  110  078 046 014
##	  106  074 042 010

import math

FoV = 27.0;
sub_increments = 0;
z = 0;
x = 0;	
y = 0;	
d = 0;  # the distance in float	
a = 0;  # the projected distance on the x-z plane

def distance2XYZ(distance_mm, spad_number):
  sub_increments = FoV/8.0;
  if spad_number == 10:
    theta = 3.0*sub_increments 
    phi = 3.0*sub_increments
  elif spad_number == 14:
    theta = 3.0*sub_increments 
    phi = 1.0*sub_increments
  elif spad_number == 245:
    theta = 3.0*sub_increments
    phi = -1.0*sub_increments
  elif spad_number == 241:
    theta = 3.0*sub_increments 
    phi = -3.0*sub_increments
  elif spad_number == 42:
    theta = 1.0*sub_increments
    phi = 3.0*sub_increments   
  elif spad_number == 46:
    theta = 1.0*sub_increments
    phi = 1.0*sub_increments     
  elif spad_number == 213:
    theta = 1.0*sub_increments 
    phi = -1.0*sub_increments      
  elif spad_number == 209:
    theta = 1.0*sub_increments 
    phi = -3.0*sub_increments
  elif spad_number == 74:
    theta = -1.0*sub_increments 
    phi = 3.0*sub_increments
  elif spad_number == 78:
    theta = -1.0*sub_increments 
    phi = 1.0*sub_increments  
  elif spad_number == 181:
    theta = -1.0*sub_increments 
    phi = -1.0*sub_increments
  elif spad_number == 117:
    theta = -1.0*sub_increments 
    phi = -3.0*sub_increments
  elif spad_number == 106:
    theta = -3.0*sub_increments 
    phi = 3.0*sub_increments
  elif spad_number == 110:
    theta = -3.0*sub_increments 
    phi = 1.0*sub_increments  
  elif spad_number == 149:
    theta = -3.0*sub_increments 
    phi = -1.0*sub_increments 
  elif spad_number == 145:
    theta = -3.0*sub_increments 
    phi = -3.0*sub_increments 
  #convert degrees to rad and calculate x,y,z
  d = distance_mm/1000 #mm to meters
  y = math.sin(math.radians(phi))*d;
  a = math.cos(math.radians(phi))*d;
  x = math.sin(math.radians(theta))*a;
  z = math.cos(math.radians(theta))*a;
  print(f"x:{x}, y:{y}, z:{z}")
  return x,y,z


distance2XYZ(100, 145)



