within GroundVehicleDynamics.Examples.Projects.Panjan;

model ProtoPanjan_008

  extends Modelica.Icons.Example;
  /****************************************/
  parameter Boolean Glb_AnimateRocketThrust = true;
  //"modelica://GroundVehicleDynamics/Examples/Projects/Panjan/tableGrd_z_xy.txt"
  //"modelica://GroundVehicleDynamics/Examples/Projects/Panjan/tableGrd_z_xy.stl"
  parameter String Glb_terrainDatFile= "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_flat.txt";
  parameter String Glb_terrainStlFile= "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_flat.stl";
  //
  parameter Modelica.Units.SI.Force Glb_RocketThrustNominal = 340;
  parameter Modelica.Units.SI.Length Glb_WheelDiameter = 3;
  parameter Modelica.Units.SI.Length Glb_WheelLength = 0.1;
  parameter Modelica.Units.SI.Length Glb_RocketLength = 0.6;
  parameter Modelica.Units.SI.Length Glb_RocketDiameter = 0.1;
  parameter Modelica.Units.SI.Angle Glb_RocketMountAngle = 1.9198621771937625;
  parameter Modelica.Units.SI.Angle Glb_StrutPhaseAngle = 0.6981317007977318;
  parameter Modelica.Units.SI.Length Glb_FuselageDiameter = 1.5;
  parameter Modelica.Units.SI.Mass Glb_FuselageMass= 3000 "total mass of fuselage";
  parameter Modelica.Units.SI.Density Glb_FuselageDensity=0.5*Glb_FuselageMass/(Modelica.Constants.pi/4*Glb_FuselageDiameter^2*Glb_FuselageLength);
  parameter Modelica.Units.SI.Length Glb_StrutLength = Glb_WheelDiameter/2;
  parameter Modelica.Units.SI.Length Glb_StrutDiameter = 0.1;
  parameter Modelica.Units.SI.Length Glb_FuselageLength = 0.6;
  //
  parameter Modelica.Units.SI.Length Glb_z0_WhlBtm_Rel = 0;
  parameter Modelica.Units.SI.Length Glb_x0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_y0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_z0_Ctr = Glb_WheelDiameter/2 + Glb_z0_WhlBtm_Rel;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_x = 0;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_y = 0;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_z = 0;
  //
  parameter Modelica.Units.SI.TranslationalSpringConstant Glb_c_GrdCntct = 1e6;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_d_GrdCntct = 1e5;
  parameter Modelica.Units.SI.Length Glb_s_rel0_GrdCntct = 0.01;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSlip = 100;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSide = 5000;
  /****************************************/
  parameter Real Glb_tblGrd[5, 5] = [0, -20, -0.9, -0.89, 100; -20, 0, 0, 0, 0; 1, 0, 0, 0, 0; 2, 0, 0, 0, 0; 100, 0, 0, 0, 0];
  
  /****************************************/
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {90, 12}, extent = {{-60, 0}, {-40, 20}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder WheelL(r = {0, 0.1, 0}, length = Glb_WheelLength, diameter = Glb_WheelDiameter, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}, innerDiameter = Glb_WheelDiameter - 0.1) annotation(
    Placement(transformation(origin = {86, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder FuselageL(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, Glb_FuselageLength, 0}, w_0_fixed = true, w_0_start = {Glb_w_0_x, Glb_w_0_y, Glb_w_0_z}, density = Glb_FuselageDensity) annotation(
    Placement(transformation(origin = {202, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenter(m = 0.001, r_0(start = {0, 0, Glb_z0_Ctr}, each fixed = true)) annotation(
    Placement(transformation(origin = {230, 314}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder FuselageR(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, Glb_FuselageLength, 0}, density = Glb_FuselageDensity) annotation(
    Placement(transformation(origin = {248, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder WheelR(diameter = Glb_WheelDiameter, length = Glb_WheelLength, r = {0, 0.1, 0}, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}, innerDiameter = Glb_WheelDiameter - 0.1) annotation(
    Placement(transformation(origin = {380, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {230, 282}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmL annotation(
    Placement(transformation(origin = {63, 206}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlL(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {79, 172}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdL(animation = false) annotation(
    Placement(transformation(origin = {89, 264}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_L annotation(
    Placement(transformation(origin = {79, 198}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {73, 252}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdR(animation = false) annotation(
    Placement(transformation(origin = {469, 264}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {453, 246}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmR annotation(
    Placement(transformation(origin = {438, 203}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlR(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {454, 166}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_R annotation(
    Placement(transformation(origin = {454, 190}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdR annotation(
    Placement(transformation(origin = {442, 140}, extent = {{-6, -6}, {6, 6}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtmL(rDisc = WheelL.diameter/2) annotation(
    Placement(transformation(origin = {41, 300}, extent = {{10, -20}, {-10, 20}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtmR(rDisc = WheelR.diameter/2) annotation(
    Placement(transformation(origin = {410, 300}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmL(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {41, 228}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmR(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {410, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelR(table = Glb_tblGrd, tableOnFile = true, tableName = "z_xy", fileName = Modelica.Utilities.Files.loadResource(Glb_terrainDatFile), delimiter = " ", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
    Placement(transformation(origin = {416, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdL annotation(
    Placement(transformation(origin = {67, 140}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelL(table = Glb_tblGrd, tableOnFile = true, tableName = "z_xy", fileName = Modelica.Utilities.Files.loadResource(Glb_terrainDatFile), delimiter = " ", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
    Placement(transformation(origin = {35, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absoluteL(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {21, 295}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheelL(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {59, 272}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absoluteR(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {429, 295}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheelR(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {430, 270}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor_ctr(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world, get_r = true, get_v = true, get_a = true, get_angles = true) annotation(
    Placement(transformation(origin = {217, 321}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles_ctr annotation(
    Placement(transformation(origin = {205, 321}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  //-----------------------------------------------------------------
  /*
  parameter Integer nGrdCellX = size(Glb_tblGrd, 1) - 2 "x 方向セル数";
  parameter Integer nGrdCellY = size(Glb_tblGrd, 2) - 2 "y 方向セル数";
  parameter Real Glb_grdZMin = VisTerrainTbl.zMin;
  parameter Real Glb_grdZMax = VisTerrainTbl.zMax;
  parameter Real Glb_grdXMin = Glb_tblGrd[2, 1] "Glb_tblGrd 内の x 最小値";
  parameter Real Glb_grdXMax = Glb_tblGrd[size(Glb_tblGrd, 1), 1] "Glb_tblGrd 内の x 最大値";
  parameter Real Glb_grdYMin = Glb_tblGrd[1, 2] "Glb_tblGrd 内の Y 最小値";
  parameter Real Glb_grdYMax = Glb_tblGrd[1, size(Glb_tblGrd, 2)] "Glb_tblGrd 内の Y 最大値";
  */
  //-----------------------------------------------------------------
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(phi(fixed = false, displayUnit = "deg"), useAxisFlange = false, animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {230, 340}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenterNoRot(m = 0.0, animation = false) annotation(
    Placement(transformation(origin = {244, 358}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles_ctrNoRot annotation(
    Placement(transformation(origin = {219, 363}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor_ctrNoRot(animation = false, get_a = true, get_angles = true, get_r = true, get_v = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {229, 363}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL1(color = {0, 255, 0}, diameter = Glb_StrutDiameter, r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false, length(displayUnit = "m")) annotation(
    Placement(transformation(origin = {0, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL1(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {13, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL1(angle = 1*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {0, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL1(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {0, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {40, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {53, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL(angle = 0*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {40, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {40, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, color = {255, 255, 255}, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {60, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL1(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {20, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL2(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-40, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL2(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-27, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL2(angle = 2*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-40, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL2(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-40, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL3(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-80, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL3(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-67, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL3(angle = 3*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-80, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL3(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-80, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL4(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-120, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL4(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-107, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL4(angle = 4*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-120, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL4(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-120, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL2(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {-20, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL3(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {-60, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL4(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-100, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Gain gain[3](k = {-2, 0, 0}) annotation(
    Placement(transformation(origin = {160, 439}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
  Modelica.Blocks.Math.Add addThrustL[3] annotation(
    Placement(transformation(origin = {60, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {57, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL1[3] annotation(
    Placement(transformation(origin = {20, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise1[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {17, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL2[3] annotation(
    Placement(transformation(origin = {-20, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-23, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL3[3] annotation(
    Placement(transformation(origin = {-60, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise3[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-63, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL4[3] annotation(
    Placement(transformation(origin = {-100, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise4[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-103, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL5(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-160, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL5(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-147, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL5(angle = 5*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-160, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL5(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-160, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL5(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-140, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL5[3] annotation(
    Placement(transformation(origin = {-140, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise5[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-143, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL6(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-200, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL6(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-187, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL6(angle = 6*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-200, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL6(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-200, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL6(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-180, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL6[3] annotation(
    Placement(transformation(origin = {-180, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise6[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-183, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL7(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-240, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL7(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-227, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutL7(angle = 7*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-240, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL7(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-240, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL7(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-220, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL7[3] annotation(
    Placement(transformation(origin = {-220, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise7[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-223, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutL8(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-280, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketL8(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-267, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut8(angle = 8*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-280, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketL8(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-280, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketL8(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-260, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustL8[3] annotation(
    Placement(transformation(origin = {-260, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise8[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-263, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR1(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {720, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR1(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {733, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR1(angle = 1*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {720, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR1(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {720, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {760, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {773, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR(angle = 0*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {760, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {760, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {780, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR1(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {740, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR2(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {680, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR2(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {693, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR2(angle = 2*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {680, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR2(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {680, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR3(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {640, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR3(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {653, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR3(angle = 3*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {640, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR3(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {640, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR4(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {600, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR4(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {613, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR4(angle = 4*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {600, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR4(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {600, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR2(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {700, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR3(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {660, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR4(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {620, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR[3] annotation(
    Placement(transformation(origin = {780, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise9[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {777, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR1[3] annotation(
    Placement(transformation(origin = {740, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise11[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {737, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR2[3] annotation(
    Placement(transformation(origin = {700, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise21[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {697, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR3[3] annotation(
    Placement(transformation(origin = {660, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise31[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {657, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR4[3] annotation(
    Placement(transformation(origin = {620, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise41[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {617, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR5(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {560, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR5(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {573, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR5(angle = 5*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {560, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR5(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {560, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR5(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {580, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR5[3] annotation(
    Placement(transformation(origin = {580, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise51[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {577, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR6(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {520, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR6(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {533, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR6(angle = 6*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {520, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR6(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {520, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR6(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {540, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR6[3] annotation(
    Placement(transformation(origin = {540, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise61[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {537, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR7(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {480, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR7(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {493, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR7(angle = 7*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {480, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR7(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {480, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR7(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {500, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR7[3] annotation(
    Placement(transformation(origin = {500, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise71[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {497, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StruR8(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {440, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR8(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {453, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR8(angle = 8*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {440, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR8(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {440, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR8(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {460, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add addThrustR8[3] annotation(
    Placement(transformation(origin = {460, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise81[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {457, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Sources.Trapezoid trapezoid_RelRocketThrust[3](amplitude = {1, 0, 0}, rising = {0.5, 0, 0}, width = {20, 0, 0}, falling = {0.5, 0, 0}, period = {40, 0, 0}, nperiod = {1, 0, 0}, offset = {0, 0, 0}, startTime = {0.1, 0, 0}) annotation(
    Placement(transformation(origin = {137, 493}, extent = {{-7, -7}, {7, 7}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtmL1(rDisc = WheelL.diameter/2) annotation(
    Placement(transformation(origin = {126, 300}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmL1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {126, 228}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelL1(table = Glb_tblGrd, tableOnFile = true, tableName = "z_xy", fileName = Modelica.Utilities.Files.loadResource(Glb_terrainDatFile), delimiter = " ", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
    Placement(transformation(origin = {120, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmL1 annotation(
    Placement(transformation(origin = {142, 206}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlL1(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {158, 172}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_L1 annotation(
    Placement(transformation(origin = {158, 198}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdL1 annotation(
    Placement(transformation(origin = {146, 140}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdL1(animation = false) annotation(
    Placement(transformation(origin = {176, 264}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const1[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {161, 252}, extent = {{-4, -4}, {4, 4}})));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheelL1(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {146, 272}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdR1(animation = false) annotation(
    Placement(transformation(origin = {359, 264}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const21[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {343, 246}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmR1 annotation(
    Placement(transformation(origin = {328, 203}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlR1(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {344, 166}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_R1 annotation(
    Placement(transformation(origin = {344, 190}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdR1 annotation(
    Placement(transformation(origin = {332, 140}, extent = {{-6, -6}, {6, 6}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtmR1(rDisc = WheelR.diameter/2) annotation(
    Placement(transformation(origin = {300, 300}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmR1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {300, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelR1(table = Glb_tblGrd, tableOnFile = true, tableName = "z_xy", fileName = Modelica.Utilities.Files.loadResource(Glb_terrainDatFile), delimiter = " ", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) annotation(
    Placement(transformation(origin = {306, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absoluteR1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {319, 295}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheelR1(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {320, 270}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape VisTerrainData(shapeType = Glb_terrainStlFile, length = 1, width = 1, height = 1, color = {255, 253, 208}, specularCoefficient = 0.8)  annotation(
    Placement(transformation(origin = {66, 102}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const_RocketThrustNominal[3](k = {Glb_RocketThrustNominal, 0, 0})  annotation(
    Placement(transformation(origin = {137, 517}, extent = {{-7, -7}, {7, 7}})));
  Modelica.Blocks.Math.Product product[3] annotation(
    Placement(transformation(origin = {160, 466}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape VisXaxis(shapeType = "cylinder", length = 200, width = 0.05, height = 0.05, color = {255, 255, 255})  annotation(
    Placement(transformation(origin = {90, 76}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape VisYaxis(color = {255, 255, 255}, height = 0.05, length = 100, shapeType = "cylinder", width = 0.05, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}) annotation(
    Placement(transformation(origin = {90, 54}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape VisZaxis(color = {255, 255, 255}, height = 0.05, length = 5, lengthDirection = {0, 0, 1}, shapeType = "cylinder", width = 0.05, widthDirection = {1, 0, 0}) annotation(
    Placement(transformation(origin = {90, 32}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(FuselageL.frame_b, WheelL.frame_a) annotation(
    Line(points = {{192, 300}, {96, 300}}, color = {95, 95, 95}));
  connect(FuselageR.frame_a, WheelR.frame_b) annotation(
    Line(points = {{258, 300}, {370, 300}}, color = {95, 95, 95}));
  connect(FuselageL.frame_a, FuselageR.frame_b) annotation(
    Line(points = {{212, 300}, {238, 300}}, color = {95, 95, 95}));
  connect(cutForce.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{230, 288}, {230, 310}}, color = {95, 95, 95}));
  connect(FuselageL.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{212, 300}, {212, 305}, {230, 305}, {230, 310}}, color = {95, 95, 95}));
  connect(elastoGapWhlL.flange_b, forceSensorGrdN_L.flange_a) annotation(
    Line(points = {{79, 182}, {79, 193}}, color = {0, 127, 0}));
  connect(positionWhlBtmL.flange, forceSensorGrdN_L.flange_b) annotation(
    Line(points = {{69, 206}, {79, 206}, {79, 203}}, color = {0, 127, 0}));
  connect(const[1].y, forceNgrdL.force[1]) annotation(
    Line(points = {{77, 252}, {89, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const[2].y, forceNgrdL.force[2]) annotation(
    Line(points = {{77, 252}, {89, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceSensorGrdN_L.f, forceNgrdL.force[3]) annotation(
    Line(points = {{84.5, 194}, {89, 194}, {89, 252}}, color = {0, 0, 127}));
  connect(const2[1].y, forceNgrdR.force[1]) annotation(
    Line(points = {{457.4, 246}, {468.8, 246}, {468.8, 252}, {469, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const2[2].y, forceNgrdR.force[2]) annotation(
    Line(points = {{457.4, 246}, {468.8, 246}, {468.8, 252}, {469, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGapWhlR.flange_b, forceSensorGrdN_R.flange_a) annotation(
    Line(points = {{454, 176}, {454, 185}}, color = {0, 127, 0}));
  connect(positionWhlBtmR.flange, forceSensorGrdN_R.flange_b) annotation(
    Line(points = {{444, 203}, {454, 203}, {454, 195}}, color = {0, 127, 0}));
  connect(forceSensorGrdN_R.f, forceNgrdR.force[3]) annotation(
    Line(points = {{459.5, 186}, {459.5, 185.5}, {469.5, 185.5}, {469.5, 252}, {469, 252}}, color = {0, 0, 127}));
  connect(positionGrdR.flange, elastoGapWhlR.flange_a) annotation(
    Line(points = {{448, 140}, {454, 140}, {454, 156}}, color = {0, 127, 0}));
  connect(WheelL.frame_b, discEdgeBtmL.frame_a) annotation(
    Line(points = {{76, 300}, {41, 300}}, color = {95, 95, 95}));
  connect(forceNgrdL.frame_b, discEdgeBtmL.frame_btm) annotation(
    Line(points = {{89, 274}, {89, 280}, {41, 280}}, color = {95, 95, 95}));
  connect(WheelR.frame_a, discEdgeBtmR.frame_a) annotation(
    Line(points = {{390, 300}, {410, 300}}, color = {95, 95, 95}));
  connect(forceNgrdR.frame_b, discEdgeBtmR.frame_btm) annotation(
    Line(points = {{469, 274}, {469, 280}, {410, 280}}, color = {95, 95, 95}));
  connect(discEdgeBtmL.frame_btm, absolutePositionWhlBtmL.frame_a) annotation(
    Line(points = {{41, 280}, {41, 234}}, color = {95, 95, 95}));
  connect(discEdgeBtmR.frame_btm, absolutePositionWhlBtmR.frame_a) annotation(
    Line(points = {{410, 280}, {410, 232}}, color = {95, 95, 95}));
  connect(absolutePositionWhlBtmR.r[1], Table_zGrd_WheelR.u1) annotation(
    Line(points = {{410, 219.4}, {410, 202.6}, {422, 202.6}, {422, 180.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR.r[2], Table_zGrd_WheelR.u2) annotation(
    Line(points = {{410, 219.4}, {410, 180.4}}, color = {0, 0, 127}));
  connect(positionGrdR.s_ref, Table_zGrd_WheelR.y) annotation(
    Line(points = {{434.8, 140}, {415.6, 140}, {415.6, 157}, {415.8, 157}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmL.r[3], positionWhlBtmL.s_ref) annotation(
    Line(points = {{41, 221.4}, {41, 206.4}, {56, 206.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR.r[3], positionWhlBtmR.s_ref) annotation(
    Line(points = {{410, 219.4}, {410, 203.4}, {431, 203.4}}, color = {0, 0, 127}));
  connect(positionGrdL.flange, elastoGapWhlL.flange_a) annotation(
    Line(points = {{73, 140}, {79, 140}, {79, 162}}, color = {0, 127, 0}));
  connect(absolutePositionWhlBtmL.r[1], Table_zGrd_WheelL.u1) annotation(
    Line(points = {{41, 221.4}, {41, 180.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmL.r[2], Table_zGrd_WheelL.u2) annotation(
    Line(points = {{41, 221.4}, {41, 216.4}, {29, 216.4}, {29, 180.4}}, color = {0, 0, 127}));
  connect(Table_zGrd_WheelL.y, positionGrdL.s_ref) annotation(
    Line(points = {{35, 157}, {35, 140}, {60, 140}}, color = {0, 0, 127}));
  connect(discEdgeBtmL.frame_a, w_absoluteL.frame_a) annotation(
    Line(points = {{41, 300}, {21, 300}}, color = {95, 95, 95}));
  connect(FtWheelL.frame_b, discEdgeBtmL.frame_btm) annotation(
    Line(points = {{59, 270}, {59, 280}, {41, 280}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_L.f, FtWheelL.u_Fn) annotation(
    Line(points = {{84.5, 194}, {89, 194}, {89, 242}, {59, 242}, {59, 267}}, color = {0, 0, 127}));
  connect(discEdgeBtmR.frame_a, w_absoluteR.frame_a) annotation(
    Line(points = {{410, 300}, {429, 300}}, color = {95, 95, 95}));
  connect(discEdgeBtmR.frame_btm, FtWheelR.frame_b) annotation(
    Line(points = {{410, 280}, {430, 280}, {430, 268}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_R.f, FtWheelR.u_Fn) annotation(
    Line(points = {{459.5, 186}, {468, 186}, {468, 239}, {430, 239}, {430, 265}}, color = {0, 0, 127}));
  connect(bodyCenter.frame_a, absoluteSensor_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {217, 310}, {217, 316}}, color = {95, 95, 95}));
  connect(bodyCenter.frame_a, absoluteAngles_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {205, 310}, {205, 316}}, color = {95, 95, 95}));
  connect(w_absoluteR.w[2], FtWheelR.u_wRoll) annotation(
    Line(points = {{429, 289.5}, {429, 286}, {416, 286}, {416, 274}}, color = {0, 0, 127}));
  connect(FuselageL.frame_a, revolute.frame_a) annotation(
    Line(points = {{212, 300}, {224, 300}, {224, 340}}, color = {95, 95, 95}));
  connect(revolute.frame_b, bodyCenterNoRot.frame_a) annotation(
    Line(points = {{236, 340}, {244, 340}, {244, 354}}, color = {95, 95, 95}));
  connect(bodyCenterNoRot.frame_a, absoluteSensor_ctrNoRot.frame_a) annotation(
    Line(points = {{244, 354}, {229, 354}, {229, 358}}, color = {95, 95, 95}));
  connect(bodyCenterNoRot.frame_a, absoluteAngles_ctrNoRot.frame_a) annotation(
    Line(points = {{244, 354}, {219, 354}, {219, 358}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL1.frame_a, WheelL.frame_b) annotation(
    Line(points = {{0, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL1.frame_b, StrutL1.frame_a) annotation(
    Line(points = {{0, 336}, {0, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL1.frame_a, StrutL1.frame_b) annotation(
    Line(points = {{0, 359}, {0, 354}}, color = {95, 95, 95}));
  connect(RocketL1.frame_a, fixedRot_RocketL1.frame_b) annotation(
    Line(points = {{6, 367}, {-0 - 16, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL.frame_b, StrutL.frame_a) annotation(
    Line(points = {{40, 336}, {40, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL.frame_a, StrutL.frame_b) annotation(
    Line(points = {{40, 359}, {40, 354}}, color = {95, 95, 95}));
  connect(RocketL.frame_a, fixedRot_RocketL.frame_b) annotation(
    Line(points = {{46, 367}, {40, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL.frame_a, WheelL.frame_b) annotation(
    Line(points = {{40, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fRocketL.frame_b, RocketL.frame_b) annotation(
    Line(points = {{60, 380}, {60, 367}}, color = {95, 95, 95}));
  connect(RocketL1.frame_b, fRocketL1.frame_b) annotation(
    Line(points = {{20, 367}, {20, 380}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL2.frame_b, StrutL2.frame_a) annotation(
    Line(points = {{-40, 336}, {-40, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL2.frame_a, StrutL2.frame_b) annotation(
    Line(points = {{-40, 359}, {-40, 354}}, color = {95, 95, 95}));
  connect(RocketL2.frame_a, fixedRot_RocketL2.frame_b) annotation(
    Line(points = {{-34, 367}, {-40, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL2.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-40, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL3.frame_b, StrutL3.frame_a) annotation(
    Line(points = {{-80, 336}, {-80, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL3.frame_a, StrutL3.frame_b) annotation(
    Line(points = {{-80, 359}, {-80, 354}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL3.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-80, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL4.frame_b, StrutL4.frame_a) annotation(
    Line(points = {{-120, 336}, {-120, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL4.frame_a, StrutL4.frame_b) annotation(
    Line(points = {{-120, 359}, {-120, 354}}, color = {95, 95, 95}));
  connect(RocketL4.frame_a, fixedRot_RocketL4.frame_b) annotation(
    Line(points = {{-114, 367}, {-120, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL4.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-120, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fRocketL2.frame_b, RocketL2.frame_b) annotation(
    Line(points = {{-20, 380}, {-20, 367}}, color = {95, 95, 95}));
  connect(fRocketL3.frame_b, RocketL3.frame_b) annotation(
    Line(points = {{-60, 380}, {-60, 367}}, color = {95, 95, 95}));
  connect(RocketL4.frame_b, fRocketL4.frame_b) annotation(
    Line(points = {{-100, 367}, {-100, 380}}, color = {95, 95, 95}));
  connect(RocketL3.frame_a, fixedRot_RocketL3.frame_b) annotation(
    Line(points = {{-74, 367}, {-80, 367}}, color = {95, 95, 95}));
  connect(addThrustL.u1, gain.y) annotation(
    Line(points = {{63, 406}, {63, 436}, {160, 436}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL.u2, const_Noise.y) annotation(
    Line(points = {{57, 406}, {57, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL.y, fRocketL.force) annotation(
    Line(points = {{60, 394.5}, {60, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const_Noise1.y, addThrustL1.u2) annotation(
    Line(points = {{17, 409.7}, {17, 405.7}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL1.y, fRocketL1.force) annotation(
    Line(points = {{20, 394.5}, {20, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL1.u1) annotation(
    Line(points = {{160, 436}, {23, 436}, {23, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL2.u2, const_Noise2.y) annotation(
    Line(points = {{-23, 406}, {-23, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL2.y, fRocketL2.force) annotation(
    Line(points = {{-20, 394.5}, {-20, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL2.u1) annotation(
    Line(points = {{160, 436}, {-17, 436}, {-17, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL3.u2, const_Noise3.y) annotation(
    Line(points = {{-63, 406}, {-63, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL3.y, fRocketL3.force) annotation(
    Line(points = {{-60, 394.5}, {-60, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL3.u1) annotation(
    Line(points = {{160, 436}, {-57, 436}, {-57, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL4.u2, const_Noise4.y) annotation(
    Line(points = {{-103, 406}, {-103, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL4.y, fRocketL4.force) annotation(
    Line(points = {{-100, 394.5}, {-100, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL4.u1) annotation(
    Line(points = {{160, 436}, {-97, 436}, {-97, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutL5.frame_b, StrutL5.frame_a) annotation(
    Line(points = {{-160, 336}, {-160, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL5.frame_a, StrutL5.frame_b) annotation(
    Line(points = {{-160, 359}, {-160, 354}}, color = {95, 95, 95}));
  connect(RocketL5.frame_a, fixedRot_RocketL5.frame_b) annotation(
    Line(points = {{-154, 367}, {-160, 367}}, color = {95, 95, 95}));
  connect(fRocketL5.frame_b, RocketL5.frame_b) annotation(
    Line(points = {{-140, 380}, {-140, 367}}, color = {95, 95, 95}));
  connect(addThrustL5.u2, const_Noise5.y) annotation(
    Line(points = {{-143, 406}, {-143, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL5.y, fRocketL5.force) annotation(
    Line(points = {{-140, 394.5}, {-140, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutL6.frame_b, StrutL6.frame_a) annotation(
    Line(points = {{-200, 336}, {-200, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL6.frame_a, StrutL6.frame_b) annotation(
    Line(points = {{-200, 359}, {-200, 354}}, color = {95, 95, 95}));
  connect(RocketL6.frame_a, fixedRot_RocketL6.frame_b) annotation(
    Line(points = {{-194, 367}, {-200, 367}}, color = {95, 95, 95}));
  connect(fRocketL6.frame_b, RocketL6.frame_b) annotation(
    Line(points = {{-180, 380}, {-180, 367}}, color = {95, 95, 95}));
  connect(addThrustL6.u2, const_Noise6.y) annotation(
    Line(points = {{-183, 406}, {-183, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL6.y, fRocketL6.force) annotation(
    Line(points = {{-180, 394.5}, {-180, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutL5.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-160, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutL6.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-200, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(gain.y, addThrustL5.u1) annotation(
    Line(points = {{160, 436}, {-137, 436}, {-137, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL6.u1) annotation(
    Line(points = {{160, 436}, {-177, 436}, {-177, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutL7.frame_b, StrutL7.frame_a) annotation(
    Line(points = {{-240, 336}, {-240, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL7.frame_a, StrutL7.frame_b) annotation(
    Line(points = {{-240, 359}, {-240, 354}}, color = {95, 95, 95}));
  connect(RocketL7.frame_a, fixedRot_RocketL7.frame_b) annotation(
    Line(points = {{-234, 367}, {-240, 367}}, color = {95, 95, 95}));
  connect(fRocketL7.frame_b, RocketL7.frame_b) annotation(
    Line(points = {{-220, 380}, {-220, 367}}, color = {95, 95, 95}));
  connect(addThrustL7.u2, const_Noise7.y) annotation(
    Line(points = {{-223, 406}, {-223, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL7.y, fRocketL7.force) annotation(
    Line(points = {{-220, 394.5}, {-220, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut8.frame_b, StrutL8.frame_a) annotation(
    Line(points = {{-280, 336}, {-280, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketL8.frame_a, StrutL8.frame_b) annotation(
    Line(points = {{-280, 359}, {-280, 354}}, color = {95, 95, 95}));
  connect(RocketL8.frame_a, fixedRot_RocketL8.frame_b) annotation(
    Line(points = {{-274, 367}, {-280, 367}}, color = {95, 95, 95}));
  connect(fRocketL8.frame_b, RocketL8.frame_b) annotation(
    Line(points = {{-260, 380}, {-260, 367}}, color = {95, 95, 95}));
  connect(addThrustL8.u2, const_Noise8.y) annotation(
    Line(points = {{-263, 406}, {-263, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustL8.y, fRocketL8.force) annotation(
    Line(points = {{-260, 394.5}, {-260, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutL7.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-240, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut8.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-280, 328}, {76, 328}, {76, 300}}, color = {95, 95, 95}));
  connect(gain.y, addThrustL7.u1) annotation(
    Line(points = {{160, 436}, {-217, 436}, {-217, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustL8.u1) annotation(
    Line(points = {{160, 436}, {-257, 436}, {-257, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR1.frame_b, StrutR1.frame_a) annotation(
    Line(points = {{720, 336}, {720, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR1.frame_a, StrutR1.frame_b) annotation(
    Line(points = {{720, 359}, {720, 354}}, color = {95, 95, 95}));
  connect(RocketR1.frame_a, fixedRot_RocketR1.frame_b) annotation(
    Line(points = {{726, 367}, {720, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR.frame_b, StrutR.frame_a) annotation(
    Line(points = {{760, 336}, {760, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR.frame_a, StrutR.frame_b) annotation(
    Line(points = {{760, 359}, {760, 354}}, color = {95, 95, 95}));
  connect(RocketR.frame_a, fixedRot_RocketR.frame_b) annotation(
    Line(points = {{766, 367}, {760, 367}}, color = {95, 95, 95}));
  connect(fRocketR.frame_b, RocketR.frame_b) annotation(
    Line(points = {{780, 380}, {780, 367}}, color = {95, 95, 95}));
  connect(RocketR1.frame_b, fRocketR1.frame_b) annotation(
    Line(points = {{740, 367}, {740, 380}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR2.frame_b, StrutR2.frame_a) annotation(
    Line(points = {{680, 336}, {680, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR2.frame_a, StrutR2.frame_b) annotation(
    Line(points = {{680, 359}, {680, 354}}, color = {95, 95, 95}));
  connect(RocketR2.frame_a, fixedRot_RocketR2.frame_b) annotation(
    Line(points = {{686, 367}, {680, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR3.frame_b, StrutR3.frame_a) annotation(
    Line(points = {{640, 336}, {640, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR3.frame_a, StrutR3.frame_b) annotation(
    Line(points = {{640, 359}, {640, 354}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR4.frame_b, StrutR4.frame_a) annotation(
    Line(points = {{600, 336}, {600, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR4.frame_a, StrutR4.frame_b) annotation(
    Line(points = {{600, 359}, {600, 354}}, color = {95, 95, 95}));
  connect(RocketR4.frame_a, fixedRot_RocketR4.frame_b) annotation(
    Line(points = {{606, 367}, {600, 367}}, color = {95, 95, 95}));
  connect(fRocketR2.frame_b, RocketR2.frame_b) annotation(
    Line(points = {{700, 380}, {700, 367}}, color = {95, 95, 95}));
  connect(fRocketR3.frame_b, RocketR3.frame_b) annotation(
    Line(points = {{660, 380}, {660, 367}}, color = {95, 95, 95}));
  connect(RocketR4.frame_b, fRocketR4.frame_b) annotation(
    Line(points = {{620, 367}, {620, 380}}, color = {95, 95, 95}));
  connect(RocketR3.frame_a, fixedRot_RocketR3.frame_b) annotation(
    Line(points = {{646, 367}, {640, 367}}, color = {95, 95, 95}));
  connect(addThrustR.u2, const_Noise9.y) annotation(
    Line(points = {{777, 406}, {777, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR.y, fRocketR.force) annotation(
    Line(points = {{780, 394.5}, {780, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const_Noise11.y, addThrustR1.u2) annotation(
    Line(points = {{737, 409.7}, {737, 405.7}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR1.y, fRocketR1.force) annotation(
    Line(points = {{740, 394.5}, {740, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR2.u2, const_Noise21.y) annotation(
    Line(points = {{697, 406}, {697, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR2.y, fRocketR2.force) annotation(
    Line(points = {{700, 394.5}, {700, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR3.u2, const_Noise31.y) annotation(
    Line(points = {{657, 406}, {657, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR3.y, fRocketR3.force) annotation(
    Line(points = {{660, 394.5}, {660, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR4.u2, const_Noise41.y) annotation(
    Line(points = {{617, 406}, {617, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR4.y, fRocketR4.force) annotation(
    Line(points = {{620, 394.5}, {620, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR5.frame_b, StrutR5.frame_a) annotation(
    Line(points = {{560, 336}, {560, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR5.frame_a, StrutR5.frame_b) annotation(
    Line(points = {{560, 359}, {560, 354}}, color = {95, 95, 95}));
  connect(RocketR5.frame_a, fixedRot_RocketR5.frame_b) annotation(
    Line(points = {{566, 367}, {560, 367}}, color = {95, 95, 95}));
  connect(fRocketR5.frame_b, RocketR5.frame_b) annotation(
    Line(points = {{580, 380}, {580, 367}}, color = {95, 95, 95}));
  connect(addThrustR5.u2, const_Noise51.y) annotation(
    Line(points = {{577, 406}, {577, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR5.y, fRocketR5.force) annotation(
    Line(points = {{580, 394.5}, {580, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR6.frame_b, StrutR6.frame_a) annotation(
    Line(points = {{520, 336}, {520, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR6.frame_a, StrutR6.frame_b) annotation(
    Line(points = {{520, 359}, {520, 354}}, color = {95, 95, 95}));
  connect(RocketR6.frame_a, fixedRot_RocketR6.frame_b) annotation(
    Line(points = {{526, 367}, {520, 367}}, color = {95, 95, 95}));
  connect(fRocketR6.frame_b, RocketR6.frame_b) annotation(
    Line(points = {{540, 380}, {540, 367}}, color = {95, 95, 95}));
  connect(addThrustR6.u2, const_Noise61.y) annotation(
    Line(points = {{537, 406}, {537, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR6.y, fRocketR6.force) annotation(
    Line(points = {{540, 394.5}, {540, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR7.frame_b, StrutR7.frame_a) annotation(
    Line(points = {{480, 336}, {480, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR7.frame_a, StrutR7.frame_b) annotation(
    Line(points = {{480, 359}, {480, 354}}, color = {95, 95, 95}));
  connect(RocketR7.frame_a, fixedRot_RocketR7.frame_b) annotation(
    Line(points = {{486, 367}, {480, 367}}, color = {95, 95, 95}));
  connect(fRocketR7.frame_b, RocketR7.frame_b) annotation(
    Line(points = {{500, 380}, {500, 367}}, color = {95, 95, 95}));
  connect(addThrustR7.u2, const_Noise71.y) annotation(
    Line(points = {{497, 406}, {497, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR7.y, fRocketR7.force) annotation(
    Line(points = {{500, 394.5}, {500, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR8.frame_b, StruR8.frame_a) annotation(
    Line(points = {{440, 336}, {440, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR8.frame_a, StruR8.frame_b) annotation(
    Line(points = {{440, 359}, {440, 354}}, color = {95, 95, 95}));
  connect(RocketR8.frame_a, fixedRot_RocketR8.frame_b) annotation(
    Line(points = {{446, 367}, {440, 367}}, color = {95, 95, 95}));
  connect(fRocketR8.frame_b, RocketR8.frame_b) annotation(
    Line(points = {{460, 380}, {460, 367}}, color = {95, 95, 95}));
  connect(addThrustR8.u2, const_Noise81.y) annotation(
    Line(points = {{457, 406}, {457, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(addThrustR8.y, fRocketR8.force) annotation(
    Line(points = {{460, 394.5}, {460, 391}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR8.frame_a, WheelR.frame_a) annotation(
    Line(points = {{440, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR7.frame_a, WheelR.frame_a) annotation(
    Line(points = {{480, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR6.frame_a, WheelR.frame_a) annotation(
    Line(points = {{520, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR5.frame_a, WheelR.frame_a) annotation(
    Line(points = {{560, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR4.frame_a, WheelR.frame_a) annotation(
    Line(points = {{600, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR3.frame_a, WheelR.frame_a) annotation(
    Line(points = {{640, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR2.frame_a, WheelR.frame_a) annotation(
    Line(points = {{680, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR1.frame_a, WheelR.frame_a) annotation(
    Line(points = {{720, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(gain.y, addThrustR8.u1) annotation(
    Line(points = {{160, 436}, {463, 436}, {463, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR7.u1) annotation(
    Line(points = {{160, 436}, {503, 436}, {503, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR6.u1) annotation(
    Line(points = {{160, 436}, {543, 436}, {543, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR5.u1) annotation(
    Line(points = {{160, 436}, {583, 436}, {583, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR4.u1) annotation(
    Line(points = {{160, 436}, {623, 436}, {623, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR3.u1) annotation(
    Line(points = {{160, 436}, {663, 436}, {663, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR2.u1) annotation(
    Line(points = {{160, 436}, {703, 436}, {703, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR.frame_a, WheelR.frame_a) annotation(
    Line(points = {{760, 328}, {390, 328}, {390, 300}}, color = {95, 95, 95}));
  connect(gain.y, addThrustR1.u1) annotation(
    Line(points = {{160, 436}, {743, 436}, {743, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, addThrustR.u1) annotation(
    Line(points = {{160, 436}, {783, 436}, {783, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(WheelL.frame_a, discEdgeBtmL1.frame_a) annotation(
    Line(points = {{96, 300}, {126, 300}}, color = {95, 95, 95}));
  connect(absolutePositionWhlBtmL1.r[1], Table_zGrd_WheelL1.u1) annotation(
    Line(points = {{126, 221.4}, {126, 180.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmL1.r[2], Table_zGrd_WheelL1.u2) annotation(
    Line(points = {{126, 221.4}, {126, 217.2}, {114, 217.2}, {114, 180.4}}, color = {0, 0, 127}));
  connect(discEdgeBtmL1.frame_btm, absolutePositionWhlBtmL1.frame_a) annotation(
    Line(points = {{126, 280}, {126, 234}}, color = {95, 95, 95}));
  connect(elastoGapWhlL1.flange_b, forceSensorGrdN_L1.flange_a) annotation(
    Line(points = {{158, 182}, {158, 193}}, color = {0, 127, 0}));
  connect(positionWhlBtmL1.flange, forceSensorGrdN_L1.flange_b) annotation(
    Line(points = {{148, 206}, {158, 206}, {158, 203}}, color = {0, 127, 0}));
  connect(positionGrdL1.flange, elastoGapWhlL1.flange_a) annotation(
    Line(points = {{152, 140}, {158, 140}, {158, 162}}, color = {0, 127, 0}));
  connect(Table_zGrd_WheelL1.y, positionGrdL1.s_ref) annotation(
    Line(points = {{120, 157}, {120, 140}, {139, 140}}, color = {0, 0, 127}));
  connect(positionWhlBtmL1.s_ref, absolutePositionWhlBtmL1.r[3]) annotation(
    Line(points = {{135, 206}, {126, 206}, {126, 221}}, color = {0, 0, 127}));
  connect(const1[1].y, forceNgrdL1.force[1]) annotation(
    Line(points = {{165.4, 252}, {176, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const1[2].y, forceNgrdL1.force[2]) annotation(
    Line(points = {{165.4, 252}, {176, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(w_absoluteL.w[2], FtWheelL.u_wRoll) annotation(
    Line(points = {{21, 289.5}, {45, 289.5}, {45, 276}}, color = {0, 0, 127}));
  connect(w_absoluteL.w[2], FtWheelL1.u_wRoll) annotation(
    Line(points = {{21, 289.5}, {132, 289.5}, {132, 276}}, color = {0, 0, 127}));
  connect(discEdgeBtmL1.frame_btm, FtWheelL1.frame_b) annotation(
    Line(points = {{126, 280}, {146, 280}, {146, 270}}, color = {95, 95, 95}));
  connect(discEdgeBtmL1.frame_btm, forceNgrdL1.frame_b) annotation(
    Line(points = {{126, 280}, {176, 280}, {176, 274}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_L1.f, forceNgrdL1.force[3]) annotation(
    Line(points = {{163.5, 194}, {176, 194}, {176, 252}}, color = {0, 0, 127}));
  connect(forceSensorGrdN_L1.f, FtWheelL1.u_Fn) annotation(
    Line(points = {{163.5, 194}, {176, 194}, {176, 244}, {146, 244}, {146, 268}}, color = {0, 0, 127}));
  connect(const21[1].y, forceNgrdR1.force[1]) annotation(
    Line(points = {{347.4, 246}, {358.8, 246}, {358.8, 252}, {359, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const21[2].y, forceNgrdR1.force[2]) annotation(
    Line(points = {{347.4, 246}, {358.8, 246}, {358.8, 252}, {359, 252}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGapWhlR1.flange_b, forceSensorGrdN_R1.flange_a) annotation(
    Line(points = {{344, 176}, {344, 185}}, color = {0, 127, 0}));
  connect(positionWhlBtmR1.flange, forceSensorGrdN_R1.flange_b) annotation(
    Line(points = {{334, 203}, {344, 203}, {344, 195}}, color = {0, 127, 0}));
  connect(forceSensorGrdN_R1.f, forceNgrdR1.force[3]) annotation(
    Line(points = {{349.5, 186}, {349.5, 185.5}, {359.5, 185.5}, {359.5, 252}, {359, 252}}, color = {0, 0, 127}));
  connect(positionGrdR1.flange, elastoGapWhlR1.flange_a) annotation(
    Line(points = {{338, 140}, {344, 140}, {344, 156}}, color = {0, 127, 0}));
  connect(forceNgrdR1.frame_b, discEdgeBtmR1.frame_btm) annotation(
    Line(points = {{359, 274}, {359, 280}, {300, 280}}, color = {95, 95, 95}));
  connect(discEdgeBtmR1.frame_btm, absolutePositionWhlBtmR1.frame_a) annotation(
    Line(points = {{300, 280}, {300, 232}}, color = {95, 95, 95}));
  connect(absolutePositionWhlBtmR1.r[1], Table_zGrd_WheelR1.u1) annotation(
    Line(points = {{300, 219.4}, {300, 202.6}, {312, 202.6}, {312, 180.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR1.r[2], Table_zGrd_WheelR1.u2) annotation(
    Line(points = {{300, 219.4}, {300, 180.4}}, color = {0, 0, 127}));
  connect(positionGrdR1.s_ref, Table_zGrd_WheelR1.y) annotation(
    Line(points = {{324.8, 140}, {305.6, 140}, {305.6, 157}, {305.8, 157}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR1.r[3], positionWhlBtmR1.s_ref) annotation(
    Line(points = {{300, 219.4}, {300, 203.4}, {321, 203.4}}, color = {0, 0, 127}));
  connect(discEdgeBtmR1.frame_a, w_absoluteR1.frame_a) annotation(
    Line(points = {{300, 300}, {319, 300}}, color = {95, 95, 95}));
  connect(discEdgeBtmR1.frame_btm, FtWheelR1.frame_b) annotation(
    Line(points = {{300, 280}, {320, 280}, {320, 268}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_R1.f, FtWheelR1.u_Fn) annotation(
    Line(points = {{349.5, 186}, {358, 186}, {358, 239}, {320, 239}, {320, 265}}, color = {0, 0, 127}));
  connect(w_absoluteR1.w[2], FtWheelR1.u_wRoll) annotation(
    Line(points = {{319, 289.5}, {319, 286}, {306, 286}, {306, 274}}, color = {0, 0, 127}));
  connect(WheelR.frame_b, discEdgeBtmR1.frame_a) annotation(
    Line(points = {{370, 300}, {300, 300}}, color = {95, 95, 95}));
  connect(VisTerrainData.frame_a, world.frame_b) annotation(
    Line(points = {{56, 102}, {56, 22}, {50, 22}}, color = {95, 95, 95}));
  connect(product.y, gain.u) annotation(
    Line(points = {{160, 459}, {160, 442}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const_RocketThrustNominal.y, product.u1) annotation(
    Line(points = {{145, 517}, {164, 517}, {164, 473}}, color = {0, 0, 127}, thickness = 0.5));
  connect(trapezoid_RelRocketThrust.y, product.u2) annotation(
    Line(points = {{145, 493}, {156, 493}, {156, 473}}, color = {0, 0, 127}, thickness = 0.5));
  connect(world.frame_b, VisXaxis.frame_a) annotation(
    Line(points = {{50, 22}, {62, 22}, {62, 76}, {80, 76}}, color = {95, 95, 95}));
  connect(VisYaxis.frame_a, world.frame_b) annotation(
    Line(points = {{80, 54}, {62, 54}, {62, 22}, {50, 22}}, color = {95, 95, 95}));
  connect(VisZaxis.frame_a, world.frame_b) annotation(
    Line(points = {{80, 32}, {62, 32}, {62, 22}, {50, 22}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-300, 540}, {800, 0}}), graphics = {Text(origin = {170, 449}, extent = {{-46, 5}, {46, -5}}, textString = "multiply by 2 because each strut has 2 rocket motors", horizontalAlignment = TextAlignment.Left)}),
    version = "",
    uses(Modelica(version = "4.1.0")),
    experiment(StartTime = 0, StopTime = 50, Tolerance = 1e-05, Interval = 0.05),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));

end ProtoPanjan_008;