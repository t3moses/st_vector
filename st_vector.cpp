#include <st_vector.h>

  st_vector::st_vector() {}

  vector_t st_vector::apparent_from_real(vector_t rw, vector_t vow) {
/*
 * First, calculate the magnetic-north-relative apparent-wind radial-vector from the magnetic-north-relative real-wind radial-vector and the magnetic-north-relative velocity-over-ground radial-vector.  Magnetic-north-relative apparent-wind is the vector sum of real-wind and velocity-over-ground.  Velocity-over-ground is calculated from velocity-over-water taking into account leeway. Velocity-over-water and real-wind are input by the user in a simulator.  Finally, convert magnetic-north-relative apparent-wind to bow-relative apparent-wind by subtracting compass-heading from the angle.
*/
      vector_t bow_relative_aw, magnetic_north_relative_aw;
      
      magnetic_north_relative_aw = st_vector::radial_from_component( st_vector::add_component( st_vector::component_from_radial( rw ), st_vector::component_from_radial( st_vector::vog_from_real(rw, vow))));
      bow_relative_aw.v1 = magnetic_north_relative_aw.v1;
      bow_relative_aw.v2 = magnetic_north_relative_aw.v2 - vow.v2;
      if (bow_relative_aw.v2 < 0) bow_relative_aw.v2 += 360.0;
      return bow_relative_aw;
  }

  vector_t st_vector::real_from_apparent(vector_t bow_relative_aw, vector_t vow) {

/*
 * First, convert bow-relative apparent-wind to magnetic-north-relative apparent-wind by adding compass-heading to the angle.  Then calculate velocity-over-ground from velocity-over-water by taking into account leeway.  Calculate the magnetic-north-relative real-wind radial-vector from the magnetic-north-relative apparent-wind radial-vector and the magnetic-north-relative velocity-over-ground radial-vector.   Magnetic-north-relative real-wind is the vector difference between magnetic-north-relative apparent-wind and the magnetic-north-relative velocity-over-ground.
 */

      vector_t magnetic_north_relative_aw;
      
      magnetic_north_relative_aw.v1 = bow_relative_aw.v1;
      magnetic_north_relative_aw.v2 = bow_relative_aw.v2 + vow.v2;
      
      return st_vector::radial_from_component( st_vector::diff_component( st_vector::component_from_radial( magnetic_north_relative_aw ), st_vector::component_from_radial( st_vector::vog_from_apparent(bow_relative_aw, vow))));

  }


  vector_t st_vector::vog_from_apparent(vector_t aw, vector_t vow) {
/*
 * Calculate the magnetic-north-relative velocity-over-ground radial-vector from the magnetic-north-relative velocity-over-water radial-vector and the bow-relative leeway angle.  Magnetic-north-relative velocity-over-water is read from the bus as reported by the paddle-wheel and compass.  Leeway is calculated by linear segments from apparent-wind angle.
 
 * Leeway has the effect of reducing the angle between the apparent wind and the bow.
 */
  vector_t vog;
      
    double lwy = st_vector::leeway_from_apparent( aw );
    
    vog.v1 = vow.v1 / cos( RADIANS_FROM_DEGREES * lwy );
    vog.v2 = vow.v2 - lwy;
      
    if (vog.v2 >= 360.0) vog.v2 -= 360.0;
    if (vog.v2 < 0.0) vog.v2 += 360.0;
      
    return vog;

  }

  vector_t st_vector::vog_from_real(vector_t rw, vector_t vow) {
 /*
 * Calculate the magnetic-north-relative velocity-over-ground radial-vector from the magnetic-north-relative velocity-over-water radial-vector and the bow-relative leeway angle.  Magnetic-north-relative velocity-over-water is read from the bus as reported by the paddle-wheel and compass.  Leeway is calculated by linear segments from real-wind angle.
 
 * Leeway has the effect of reducing the angle between the real wind and the bow.
 */
  vector_t vog;
      
      double lwy = st_vector::leeway_from_real( rw );
      
    vog.v1 = vow.v1 / cos( RADIANS_FROM_DEGREES * lwy );
    vog.v2 = vow.v2 - lwy;
      
    if (vog.v2 >= 360.0) vog.v2 -= 360.0;
    if (vog.v2 < 0.0) vog.v2 += 360.0;
      
    return vog;
  }


double st_vector::leeway_from_apparent( vector_t aw ) {
        
  return A * exp( -B * aw.v2 );
    
}

double st_vector::leeway_from_real( vector_t rw ) {
    
      
  return A * exp( -C * rw.v2 );
  
}


  vector_t st_vector::component_from_radial(vector_t v) {

  vector_t result;

    result.v1 = v.v1 * sin( RADIANS_FROM_DEGREES * v.v2);
    result.v2 = v.v1 * cos( RADIANS_FROM_DEGREES * v.v2);
    return result;
    
  }

  vector_t st_vector::radial_from_component(vector_t v) {
  
  vector_t result;
      double quad_adjust;

      if((v.v2 >= 0 && v.v2 < MIN_NORTH) || (v.v2 <= 0 && v.v2 > -MIN_NORTH)) {
          if (v.v1 >= 0) {
            result.v1 = v.v1;
            result.v2 = 90;
          }
          else {
            result.v1 = -v.v1;
            result.v2 = 270;
          }
          return result;
      }
      
      if(v.v1 >= 0 && v.v2 >= 0) quad_adjust = 0.0; // The resulting angle is in the range 0 .. PI / 2, and arctan is in the range 0 .. PI / 2
      if(v.v1 >= 0 && v.v2 < 0) quad_adjust = 1.0; // The resulting angle is in the range PI / 2 .. PI, but arctan is in the range -PI / 2 .. 0
      if(v.v1 < 0 && v.v2 < 0) quad_adjust = 1.0; // The resulting angle is in the range PI .. 3 * PI / 2, but arctan is in the range 0 .. PI / 2
      if(v.v1 < 0 && v.v2 >= 0) quad_adjust = 2.0; // The resulting angle is in the range 3 * PI / 2 .. 2 * PI, but arctan is in the range -PI / 2 .. 0
    result.v1 = sqrt(pow(v.v1, 2) + pow(v.v2, 2));
    result.v2 = DEGREES_FROM_RADIANS * (atan(v.v1 / v.v2) + quad_adjust * PI);
    return result;
    
  }


vector_t st_vector::add_radial(vector_t va, vector_t vb) {

    return st_vector::radial_from_component( st_vector::add_component( st_vector::component_from_radial( va ), st_vector::component_from_radial( vb )));
    
}


  vector_t st_vector::add_component(vector_t va, vector_t vb) {

  vector_t result;
  
    result.v1 = va.v1 + vb.v1;
    result.v2 = va.v2 + vb.v2;
    return result;
    
  }

  vector_t st_vector::diff_component(vector_t va, vector_t vb) {

  vector_t result;
  
    result.v1 = va.v1 - vb.v1;
    result.v2 = va.v2 - vb.v2;
    return result;
    
  }
