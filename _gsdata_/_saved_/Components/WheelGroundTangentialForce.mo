within GroundVehicleDynamics.Components;

model WheelGroundTangentialForce
  import SI = Modelica.Units.SI;
  import MB = Modelica.Mechanics.MultiBody;
  import Frames = Modelica.Mechanics.MultiBody.Frames;

  extends MB.Interfaces.PartialOneFrame_b;

  Modelica.Blocks.Interfaces.RealInput u_Fn(unit="N") 
    "Normal contact force magnitude (>=0)" annotation(
    Placement(transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput u_wRoll(unit="rad/s")
    "Wheel spin angular velocity around wheel axis" annotation(
    Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  parameter SI.Length rDisc = 1 "Wheel radius";
  parameter Integer axisDisc = 2
    "Wheel spin axis in frame_b: 1=x, 2=y, 3=z";
  parameter Real mu = 0.8 "Coulomb friction coefficient";
  parameter SI.TranslationalDampingConstant kLong = 20000
    "Longitudinal no-slip constraint gain";
  parameter SI.TranslationalDampingConstant kLat = 20000
    "Lateral no-slip constraint gain";
  parameter SI.Velocity vReg = 1e-4
    "Velocity regularization threshold";
  parameter Boolean enableFrictionOnlyWhenLoaded = true
    "If true, no tangential force when Fn <= 0";

protected
  Real eAxis[3];
  Real ez[3] = {0,0,1};
  Real down[3] = {0,0,-1};

  Real a_world[3];
  Real nRaw[3];
  Real nContact[3];
  Real tRaw[3];
  Real tDir[3];
  Real sRaw[3];
  Real sDir[3];
  Real v0_world[3];

  Real stickSel;
  Real loadedSel;
  Real slipSel;

public
  SI.Velocity vRoll;
  SI.Velocity vSide;
  SI.Velocity vSlipRoll;
  SI.Force FtRoll_cmd;
  SI.Force FtSide_cmd;
  SI.Force FtRoll;
  SI.Force FtSide;
  SI.Force Ft_lim;
  SI.Force FtNeed;
  SI.Velocity slipNorm;

protected
  SI.Force FtRollSlip;
  SI.Force FtSideSlip;
  Real fTan_cmd[3];
  Real f_world[3];

equation
  eAxis = if axisDisc == 1 then {1,0,0}
          elseif axisDisc == 2 then {0,1,0}
          else {0,0,1};

  a_world = Frames.resolve1(frame_b.R, eAxis);

  nRaw = down - (down*a_world)*a_world;

  nContact = noEvent(
    if Modelica.Math.Vectors.length(nRaw) > vReg then
      nRaw / Modelica.Math.Vectors.length(nRaw)
    else
      -ez);

  tRaw = cross(a_world, nContact);

  tDir = noEvent(
    if Modelica.Math.Vectors.length(tRaw) > vReg then
      tRaw / Modelica.Math.Vectors.length(tRaw)
    else
      {1,0,0});

  sRaw = cross(nContact, tDir);

  sDir = noEvent(
    if Modelica.Math.Vectors.length(sRaw) > vReg then
      sRaw / Modelica.Math.Vectors.length(sRaw)
    else
      {0,1,0});

  v0_world = der(frame_b.r_0);

  vRoll = v0_world * tDir;
  vSide = v0_world * sDir;
  vSlipRoll = vRoll - rDisc*u_wRoll;

  FtRoll_cmd = -kLong*vSlipRoll;
  FtSide_cmd = -kLat*vSide;

  FtNeed = Modelica.Math.Vectors.length({FtRoll_cmd, FtSide_cmd});
  Ft_lim = mu*max(u_Fn, 0);

  loadedSel = if enableFrictionOnlyWhenLoaded and u_Fn > 0 then 1.0 else 0.0;
  stickSel = if loadedSel > 0 and FtNeed <= Ft_lim then 1.0 else 0.0;
  slipSel = 1.0 - stickSel;
  
  slipNorm = Modelica.Math.Vectors.length({vSlipRoll, vSide});
  
  FtRollSlip = -Ft_lim * vSlipRoll / max(Modelica.Math.Vectors.length({vSlipRoll, vSide}), vReg);
  FtSideSlip = -Ft_lim * vSide / max(Modelica.Math.Vectors.length({vSlipRoll, vSide}), vReg);

  FtRoll = loadedSel * (stickSel*FtRoll_cmd + slipSel*FtRollSlip);
  FtSide = loadedSel * (stickSel*FtSide_cmd + slipSel*FtSideSlip);

  fTan_cmd = FtRoll*tDir + FtSide*sDir;
  f_world = fTan_cmd;

  frame_b.f = Frames.resolve2(frame_b.R, f_world);
  frame_b.t = zeros(3);

annotation(
    defaultComponentName = "FtWheel",
    Icon(coordinateSystem(extent = {{-40, -20}, {240, 60}}), graphics = {Line(origin = {90, 0}, points = {{10, 0}, {90, 0}}, thickness = 2, arrow = {Arrow.None, Arrow.Open}, arrowSize = 9), Line(origin = {84.01, 33.21}, points = {{-124.013, 10.7924}, {-84.013, -11.2076}, {-44.0129, -23.2076}, {-10.0129, -31.2076}, {15.9871, -33.2076}, {41.9871, -31.2076}, {77.987, -23.2076}, {113.987, -11.2076}, {155, 10.7924}}, thickness = 4), Line(origin = {10, 26}, points = {{90, -26}, {90, 30}}, pattern = LinePattern.Dash, thickness = 2, arrow = {Arrow.None, Arrow.Open}, arrowSize = 9)}),
    Diagram(graphics));
end WheelGroundTangentialForce;