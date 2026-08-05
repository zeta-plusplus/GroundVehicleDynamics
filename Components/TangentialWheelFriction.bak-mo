within GroundVehicleDynamics.Components;

model TangentialWheelFriction

  import SI = Modelica.Units.SI;
  import MB = Modelica.Mechanics.MultiBody;
  import Frames = Modelica.Mechanics.MultiBody.Frames;
  import Types = Modelica.Mechanics.MultiBody.Types;

  extends MB.Interfaces.PartialOneFrame_b;

  Modelica.Blocks.Interfaces.RealInput Fn(unit="N")
    "Normal contact force magnitude, >= 0";
  Modelica.Blocks.Interfaces.RealInput wRoll(unit="rad/s")
    "Wheel angular velocity around disc axis";

  parameter SI.Length rDisc = 1 "Disc radius";
  parameter Integer axisDisc = 2
    "Disc axis in frame_b: 1=x, 2=y, 3=z";
  parameter Real mu = 0.8 "Coulomb friction coefficient";
  parameter SI.TranslationalDampingConstant cSlip = 500
    "Viscous slip gain";
  parameter SI.Velocity vSmall = 1e-4
    "Regularization threshold";

protected
  Real eAxis[3];
  Real ez[3] = {0,0,1};
  Real a_world[3];
  Real tDir_raw[3];
  Real tDir_world[3];
  Real v_world[3];
  Real vt;
  SI.Velocity vSlip;
  SI.Force Ft_unsat;
  SI.Force Ft;
  Real force_world[3];

equation
  
  eAxis = if axisDisc == 1 then {1,0,0}
          elseif axisDisc == 2 then {0,1,0}
          else {0,0,1};
  
  //a_world = Frames.resolve1(frame_b.R, eAxis);

  // 接線方向 = 車輪面内で、地面法線 ez と軸 a_world の両方に直交
  //tDir_raw = ez cross a_world;
  
  tDir_world =
    if Modelica.Math.Vectors.length(tDir_raw) > vSmall then
      tDir_raw / Modelica.Math.Vectors.length(tDir_raw)
    else
      {1,0,0};

  // 接触点フレームの並進速度（world座標）
  v_world = der(frame_b.r_0);

  // 接線方向速度成分
  vt = v_world * tDir_world;

  // slip速度: 地面に対する接触点の相対接線速度
  vSlip = vt - rDisc*wRoll;

  // 粘性型の候補力
  Ft_unsat = -cSlip*vSlip;

  // Coulomb上限で飽和
  Ft = max(-mu*max(Fn,0), min(mu*max(Fn,0), Ft_unsat));

  // world座標で接線力を作用
  force_world = Ft * tDir_world;

  frame_b.f = Frames.resolve2(frame_b.R, force_world);
  frame_b.t = zeros(3);

end TangentialWheelFriction;
