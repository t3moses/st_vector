#include <st_vector.h>

  st_vector::st_vector() {}

  st_vector::vector_t st_vector::aw_from_real(st_vector::vector_t rw, st_vector::vector_t vow) {
/*
 * First, calculate the magnetic-north-relative apparent-wind radial-vector from the magnetic-north-relative real-wind radial-vector and the magnetic-north-relative velocity-over-ground radial-vector.  Magnetic-north-relative apparent-wind is the vector sum of real-wind and velocity-over-ground.  Velocity-over-ground is calculated from velocity-over-water taking into account leeway. Velocity-over-water and real-wind are input by the user in a simulator.  Finally, convert magnetic-north-relative apparent-wind to bow-relative apparent-wind by subtracting compass-heading from the angle.
*/
      st_vector::vector_t bow_relative_aw, magnetic_north_relative_aw;
      
      magnetic_north_relative_aw = st_vector::radial_from_component( st_vector::add_component( st_vector::component_from_radial( rw ), st_vector::component_from_radial( st_vector::vog_from_real(rw, vow))));
      bow_relative_aw.v1 = magnetic_north_relative_aw.v1;
      bow_relative_aw.v2 = magnetic_north_relative_aw.v2 - vow.v2;
      return bow_relative_aw;
  }

  st_vector::vector_t st_vector::rw_from_apparent(st_vector::vector_t bow_relative_aw, st_vector::vector_t vow) {

/*
 * First, convert bow-relative apparent-wind to magnetic-north-relative apparent-wind by adding compass-heading to the angle.  Then calculate velocity-over-ground from velocity-over-water by taking into account leeway.  Calculate the magnetic-north-relative real-wind radial-vector from the magnetic-north-relative apparent-wind radial-vector and the magnetic-north-relative velocity-over-ground radial-vector.   Magnetic-north-relative real-wind is the vector difference between magnetic-north-relative apparent-wind and the magnetic-north-relative velocity-over-ground.
 */

      st_vector::vector_t magnetic_north_relative_aw;
      
      magnetic_north_relative_aw.v1 = bow_relative_aw.v1;
      magnetic_north_relative_aw.v2 = bow_relative_aw.v2 + vow.v2;
      
      return st_vector::radial_from_component( st_vector::diff_component( st_vector::component_from_radial( magnetic_north_relative_aw ), st_vector::component_from_radial( st_vector::vog_from_apparent(bow_relative_aw, vow))));

  }

  st_vector::vector_t st_vector::vog_from_apparent(st_vector::vector_t aw, st_vector::vector_t vow) {
/*
 * Calculate the magnetic-north-relative velocity-over-ground radial-vector from the magnetic-north-relative velocity-over-water radial-vector and the bow-relative leeway angle.  Magnetic-north-relative velocity-over-water is read from the bus as reported by the paddle-wheel and compass.  Leeway is calculated by regression from apparent-wind angle.
 */
  st_vector::vector_t vog;
  
    double lwy = LEEWAY_MAX * (A * pow(aw.v2, 3) + B * pow(aw.v2, 2) + C * aw.v2 + D);
    vog.v1 = vow.v1 / cos( RADIANS_FROM_DEGREES * lwy );
    vog.v2 = vow.v2 + lwy;
    if (vog.v2 >= 360.0) vog.v2 -= 360;
    return vog;

  }

  st_vector::vector_t st_vector::vog_from_real(st_vector::vector_t rw, st_vector::vector_t vow) {
 /*
 * Calculate the magnetic-north-relative velocity-over-ground radial-vector from the magnetic-north-relative velocity-over-water radial-vector and the bow-relative leeway angle.  Magnetic-north-relative velocity-over-water is read from the bus as reported by the paddle-wheel and compass.  Leeway is calculated by regression from real-wind minus heading angle.
 */
  st_vector::vector_t vog;
 
    double lwy = LEEWAY_MAX * (E * pow((rw.v2 - vow.v2), 3) + F * pow((rw.v2 - vow.v2), 2) + G * (rw.v2 - vow.v2) + H);
    vog.v1 = vow.v1 / cos( RADIANS_FROM_DEGREES * lwy );
    vog.v2 = vow.v2 + lwy;
    if (vog.v2 >= 360.0) vog.v2 -= 360;
    return vog;
    
  }

  st_vector::vector_t st_vector::component_from_radial(st_vector::vector_t v) {

  st_vector::vector_t result;

    result.v1 = v.v1 * sin( RADIANS_FROM_DEGREES * v.v2);
    result.v2 = v.v1 * cos( RADIANS_FROM_DEGREES * v.v2);
    return result;
    
  }

  st_vector::vector_t st_vector::radial_from_component(st_vector::vector_t v) {
  
  st_vector::vector_t result;
  uint8_t quad_adjust = 0;

    if(v.v1 > 0 && v.v2 > 0) quad_adjust = 0; // The resulting angle is in the range 0 .. PI / 2, and arctan is in the range 0 .. PI / 2
    if(v.v1 > 0 && v.v2 < 0) quad_adjust = 1; // The resulting angle is in the range PI / 2 .. PI, but arctan is in the range -PI / 2 .. 0
    if(v.v1 < 0 && v.v2 < 0) quad_adjust = 1; // The resulting angle is in the range PI .. 3 * PI / 2, but arctan is in the range 0 .. PI / 2
    if(v.v1 < 0 && v.v2 > 0) quad_adjust = 2; // The resulting angle is in the range 3 * PI / 2 .. 2 * PI, but arctan is in the range -PI / 2 .. 0
    result.v1 = sqrt(pow(v.v1, 2) + pow(v.v2, 2));
    result.v2 = DEGREES_FROM_RADIANS * (atan(v.v1 / v.v2) + quad_adjust * PI);
    return result;
    
  }

  st_vector::vector_t st_vector::add_component(st_vector::vector_t va, st_vector::vector_t vb) {

  st_vector::vector_t result;
  
    result.v1 = va.v1 + vb.v1;
    result.v2 = va.v2 + vb.v2;
    return result;
    
  }

  st_vector::vector_t st_vector::diff_component(st_vector::vector_t va, st_vector::vector_t vb) {

  st_vector::vector_t result;
  
    result.v1 = va.v1 - vb.v1;
    result.v2 = va.v2 - vb.v2;
    return result;
    
  }
