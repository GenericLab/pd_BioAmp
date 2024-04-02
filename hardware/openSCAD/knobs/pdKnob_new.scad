/* [Basic] */
//Diametre du bouton
Knob_Diameter=14.0; //[10:30]
// inner Diameter
Shaft_Diameter = 4.0;
//Hauteur du bouton
Knob_Height=14;//[10:20]
//Longueur de l'axe
Axis_Lenght=14;//[5:15]
//Arrondis
Rounded=1.0;//[1:3]
//Ajustement
Ajustment=0.05;//[0,0.1,0.2,0.3]
/* [Hidden] */
$fn=100;

rotate([180,0,0]) difference() {
  union() {
      cylinder(r1=Knob_Diameter/2+0.0, r2=Knob_Diameter/2,h=Knob_Height-Rounded+0.8);
      translate([0,0,Knob_Height-2*Rounded]) cylinder(r=Knob_Diameter/2-Rounded,h=Rounded);
      difference() {
      translate([0,0,Knob_Height-Rounded+1.0]) rotate_extrude(convexity = 10) translate([Knob_Diameter/2+0.3-Rounded,0,0])  circle(r=Rounded);
     translate([0,0,Knob_Height]) rotate([90,0,0]) cylinder(r1= 0, r2=6.3,h=Knob_Diameter/2+1);

      }
      translate([0,0,Knob_Height]) sphere(r=1.0);
      translate([0,-Knob_Diameter/2-0.0,Knob_Height]) sphere(r=1.0);
      rotate([0,0,38]) translate([0,-Knob_Diameter/2+0.7,Knob_Height-0.0]) sphere(r=1.0);
      rotate([0,0,-38]) translate([0,-Knob_Diameter/2+0.7,Knob_Height-0.0]) sphere(r=1.0);
      translate([0,-Knob_Diameter/2-0.0,4]) cylinder(r=1.0,h=Knob_Height-4);
      translate([0,-Knob_Diameter/2-0.0,Knob_Height-10]) sphere(r=1.0); //lower bubble
      translate([0,0,Knob_Height]) rotate([90,0,0]) cylinder(r=1.0,h=Knob_Diameter/2);
      
      for (i=[0:60:300]) translate([(Knob_Diameter-0.7)*sin(i)/2,(Knob_Diameter-0.7)*cos(i)/2,5])  cylinder(r1=0.7, r2=0.7*1,h=Knob_Height/1.6);
  }

  //translate([-Knob_Diameter,-Knob_Diameter/2,Knob_Height-3]) rotate([45,0,0]) cube([Knob_Diameter*2,12,10]);
  

  //for (i=[0:60:360]) translate([(Knob_Diameter+0.6)*sin(i)/2,(Knob_Diameter+0.6)*cos(i)/2,3])  cylinder(r1=0.7, r2=0.7*1,h=Knob_Height);
  difference() {
      translate([0,0,2-0.11]) cylinder(r=3.0+Ajustment,h=Axis_Lenght-3);
      //translate([-5,1.4+Ajustment,6]) cube([10,5,Axis_Lenght-2]); // D-shape
  }
  difference() {
      translate([0,0,-1.1]) cylinder(r=6+Ajustment,h=3);
      //translate([-5,-6.5-Ajustment,0]) cube([10,5,Axis_Lenght-2]);
  }
  difference() {
      translate([0,0,1.8]) cylinder(r1=6+Ajustment,r2=3+Ajustment,h=2);
      //translate([-5,-6.5-Ajustment,0]) cube([10,5,Axis_Lenght-2]);
  }
}
