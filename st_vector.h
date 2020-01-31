#ifndef SEATALK_VECTOR
  #define SEATALK_VECTOR

  #if (ARDUINO >=100)
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
    
#define RADIANS_FROM_DEGREES 0.0175
#define DEGREES_FROM_RADIANS 57.296
#define LEEWAY_MAX 10
#define A - 6.734006734 * pow(10, -7)
#define B + 3.636363636 * pow(10, -4)
#define C - 4.363636364 * pow(10, -2)
#define D + 3.819167205 * pow(10, -13)
#define E - 4.62962963· * pow(10, -7)
#define F + 0.00025
#define G - 0.03
#define H + 1.687538997 * pow(10, -13)

/*
 lwy = A x awa^3 + B x awa^2 + C x awa + D
 By polynoial regression from these points:
 
 awa     lwy
 0        0
 30      -1
 180      0
 330      1
 360      0
 
 lwy = E x (rwa - hdg) ^3 + F x (rwa - hdg) ^2 + G x (rwa - hdg) + H
 By polynoial aregression from these points:
 
 rwa-hdg lwy
 0        0
 60      -1
 180      0
 300      1
 360      0
 
*/
  class st_vector {

    public:

      struct vector_t {
        
        double v1;
        double v2;
        
      };
      
      st_vector();
      vector_t aw_from_real(vector_t rw, vector_t vow);
      vector_t rw_from_apparent(vector_t aw, vector_t vow);
    
    private:

      vector_t vog_from_apparent(vector_t aw, vector_t vow); // sog/cog calculated on basis of apparent wind.
      vector_t vog_from_real(vector_t rw, vector_t vow); // sog/cog calculated on basis of real wind and heading.
      vector_t component_from_radial(vector_t v); // Convert radial representation to component representation.
      vector_t radial_from_component(vector_t v); // Convert component representation to radial representation.
      vector_t add_component(vector_t v1, vector_t v2); // Return v1 + v2 vector sum.
      vector_t diff_component(vector_t v1, vector_t v2); // Return v1 - v2 vector difference.
      
  };
#endif
