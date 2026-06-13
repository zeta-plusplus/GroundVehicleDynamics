within GroundVehicleDynamics.Components;

model DiscEdgeBottomTranslation
  import units = Modelica.Units.SI;
  import MB = Modelica.Mechanics.MultiBody;
  import Frames = Modelica.Mechanics.MultiBody.Frames;
  
  /********************
  *********************/
  parameter units.Length rDisc = 1 "Disc radius";
  parameter Integer axisDisc = 2
    "Disc axis in frame_a: 1=x, 2=y, 3=z";
  
  
  /********************
  *********************/
  MB.Interfaces.Frame_a frame_a
    annotation (Placement(transformation(origin = {100, 0}, extent = {{-116, -16}, {-84, 16}}), iconTransformation(origin = {100, 0}, extent = {{-116, -16}, {-84, 16}})));
  
  MB.Interfaces.Frame_b frame_btm
    annotation (Placement(transformation( extent = {{84, -16}, {116, 16}}, rotation = -90), iconTransformation( extent = {{84, -16}, {116, 16}}, rotation = -90)));
protected
  Real eAxis[3];
  Real ez[3] = {0,0,1};
  Real a_world[3];
  Real rTmp[3];
  Real rDir_world[3];
  units.Position r_btm_world[3];
  units.Position r_btm_a[3];
  units.Force f_btm_a[3];
  units.Torque t_btm_a[3];

//********************************************************************************
equation
  eAxis = if axisDisc == 1 then {1,0,0}
          elseif axisDisc == 2 then {0,1,0}
          else {0,0,1};

  a_world = Frames.resolve1(frame_a.R, eAxis);

  rTmp = -ez + (a_world*ez)*a_world;

  rDir_world =
    if Modelica.Math.Vectors.length(rTmp) > 1e-10 then
      rTmp / Modelica.Math.Vectors.length(rTmp)
    else
      {0,0,-1};

  r_btm_world = rDisc * rDir_world;

  frame_btm.r_0 = frame_a.r_0 + r_btm_world;
  frame_btm.R   = frame_a.R;

  r_btm_a = Frames.resolve2(frame_a.R, r_btm_world);
  f_btm_a = Frames.resolve2(frame_a.R, frame_btm.f);
  t_btm_a = Frames.resolve2(frame_a.R, frame_btm.t);

  frame_a.f = -Frames.resolve1(frame_a.R, f_btm_a);
  frame_a.t = -Frames.resolve1(frame_a.R, t_btm_a + cross(r_btm_a, f_btm_a));
  
  
annotation(
    Icon(graphics = {Ellipse(pattern = LinePattern.Dash, extent = {{-100, 100}, {100, -100}}), Text(origin = {19, 0}, extent = {{-11, 6}, {11, -6}}, textString = "center"), Text(origin = {1, -84}, extent = {{-11, 6}, {11, -6}}, textString = "bottom"), Text(origin = {0, 112}, textColor = {0, 0, 255}, extent = {{-100, 8}, {100, -8}}, textString = "%name")}, coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Diagram(graphics));
end DiscEdgeBottomTranslation;
