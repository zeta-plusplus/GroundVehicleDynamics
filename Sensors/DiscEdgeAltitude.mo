within GroundVehicleDynamics.Sensors;

model DiscEdgeAltitude
  extends Modelica.Icons.RoundSensor;
  import units = Modelica.Units.SI;
  
  /********************
  *********************/
  parameter units.Length rDisc = 1;
  
  /********************
  *********************/
  Real a[3];
  Real ez[3] = {0,0,1};
  units.Position rTmp[3];
  units.Position r_Min_z[3];
  
  /********************
  *********************/
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Blocks.Interfaces.RealOutput r[3] annotation(
    Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput y_alt annotation(
    Placement(transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}})));
equation
  a = Modelica.Mechanics.MultiBody.Frames.resolve1(frame_a.R, {0,1,0});
  rTmp = -ez + (a*ez)*a;
  r_Min_z = rTmp/Modelica.Math.Vectors.length(rTmp);
  r = frame_a.r_0 + rDisc*r_Min_z;
  y_alt = r[3];

  frame_a.f = zeros(3);
  frame_a.t = zeros(3); 
annotation(
    Icon(graphics = {Line(origin = {-86, 0}, points = {{-16, 0}, {16, 0}}, thickness = 2), Text(origin = {0, 90}, textColor = {0, 0, 255}, extent = {{-100, 10}, {100, -10}}, textString = "%name"), Line(origin = {86, 0}, points = {{-16, 0}, {16, 0}}, thickness = 2), Line(origin = {86, -20}, points = {{-16, 20}, {-4, 20}, {-4, -20}, {16, -20}}, thickness = 2), Text(origin = {94, 17}, extent = {{-8, 5}, {8, -5}}, textString = "r"), Text(origin = {94, -57}, extent = {{-8, 5}, {8, -5}}, textString = "alt")}),
  Diagram(graphics));
end DiscEdgeAltitude;
