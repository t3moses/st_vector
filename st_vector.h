#ifndef SEATALK_VECTOR
  #define SEATALK_VECTOR

  #if (ARDUINO >=100)
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  #define RADIANS_FROM_DEGREES 0.0174533
  #define DEGREES_FROM_RADIANS 57.2958
  #define MIN_NORTH 0.001 // Avoid divide by zero.
  #define A 180 // Constants used in the cacluation of leeway.  Course = heading - leeway (mod 360).
  #define B -0.096 // Leeway is modelled as an inverse exponential.
  #define C -0.064 // A, B and C are chosen such that leeway is 10 degrees at an apparent wind angle of 30 degrees, and at a real wind angle of 45 degrees.

  struct vector_t {
  
    double v1;
    double v2;
  
  };

  class st_vector {

    public:
      
      st_vector();
      vector_t apparent_from_real(vector_t rw, vector_t vow);
      vector_t real_from_apparent(vector_t aw, vector_t vow);
      vector_t add_radial(vector_t va, vector_t vb);
    
    private:

      vector_t vog_from_apparent(vector_t aw, vector_t vow); // sog/cog calculated on basis of apparent wind.
      vector_t vog_from_real(vector_t rw, vector_t vow); // sog/cog calculated on basis of real wind and heading.
      double leeway_from_real ( vector_t rw );
      double leeway_from_apparent ( vector_t aw );
      vector_t component_from_radial(vector_t v); // Convert radial representation to component representation.
      vector_t radial_from_component(vector_t v); // Convert component representation to radial representation.
      vector_t add_component(vector_t v1, vector_t v2); // Return v1 + v2 vector sum.
      vector_t diff_component(vector_t v1, vector_t v2); // Return v1 - v2 vector difference.
      
  };
#endif
