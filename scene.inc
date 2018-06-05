sky_sphere{
  pigment {
    color rgb<1,1,1>
  }
  scale 0.3
  rotate <0,3,3>
}


global_settings {
  ambient_light rgb<.2, .2, .2>
  //charset "utf8"
}
// light_source{<10,20,15>
//   // Sun
//   color rgb<1,1,1>
//   parallel
//   jitter
//   adaptive 1
// }

light_source{<6,12,6>
  // Sun
  color rgb<.75,.75,.75>

  //color rgb<1,1,1>
  //area_light <2, 0, 2>, <0, 2, 2>, 10, 10
  area_light <3, 0, 3>, <0, 3, 3>, 10, 10

  parallel
  jitter
  //fade_distance 10
  //fade_power 1
  //adaptive 10
}

// #include "shapes.inc"
// #include "colors.inc"
// #include "textures.inc"
// #include "metals.inc"

// #declare Teapot_Texture = T_Chrome_5E;
// #declare Teapot_Orientation = <-90, 90, 0>;

// object {
//   #include "/usr/share/povray-3.7/scenes/advanced/teapot/teapot.inc"
//   scale .08

//   translate <.75, .762, 0>
// }
