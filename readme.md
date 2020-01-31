This Seatalk library converts between real and apparent wind strength and direction for a boat under sail.

The library exposes two functions;

vector_t aw_from_real(vector_t rw, vector_t vow);
vector_t rw_from_apparent(vector_t aw, vector_t vow);

and one type:

struct vector_t {
  
  double v1; // Strength
  double v2; // Upwind angle clockwise from magnetic north (in the case of rw and vow) and from the bow (in the case of aw).
  
};

The functions take leeway into account, using estimation and polynomial regression.

The library has only been tested on an Arduino Uno R3 with a 16 MHz system clock.
