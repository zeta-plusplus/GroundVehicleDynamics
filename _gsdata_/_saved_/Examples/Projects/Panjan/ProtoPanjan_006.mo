within GroundVehicleDynamics.Examples.Projects.Panjan;

model ProtoPanjan_006
  extends Modelica.Icons.Example;
  /****************************************/
  parameter Boolean Glb_AnimateRocketThrust = true;
  //
  parameter Modelica.Units.SI.Force Glb_RocketThrustNominal = 1000;
  parameter Modelica.Units.SI.Length Glb_WheelDiameter = 3;
  parameter Modelica.Units.SI.Length Glb_WheelLength = 0.1;
  parameter Modelica.Units.SI.Length Glb_RocketLength = 0.6;
  parameter Modelica.Units.SI.Length Glb_RocketDiameter = 0.1;
  parameter Modelica.Units.SI.Angle Glb_RocketMountAngle = 1.9198621771937625;
  parameter Modelica.Units.SI.Angle Glb_StrutPhaseAngle = 0.6981317007977318;
  parameter Modelica.Units.SI.Length Glb_FuselageDiameter = 1;
  parameter Modelica.Units.SI.Length Glb_StrutLength = Glb_WheelDiameter/2;
  parameter Modelica.Units.SI.Length Glb_StrutDiameter = 0.1;
  parameter Modelica.Units.SI.Length Glb_FuselageLength = 1;
  //
  parameter Modelica.Units.SI.Length Glb_z0_WhlBtm_Rel = 0.1;
  parameter Modelica.Units.SI.Length Glb_x0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_y0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_z0_Ctr = Glb_WheelDiameter/2 + Glb_grdZMax + Glb_z0_WhlBtm_Rel;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_x = 0;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_y = 0;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_z = 0;
  //
  parameter Modelica.Units.SI.TranslationalSpringConstant Glb_c_GrdCntct = 1e7;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_d_GrdCntct = 1e6;
  parameter Modelica.Units.SI.Length Glb_s_rel0_GrdCntct = 1e-5;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSlip = 100;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSide = 5000;
  /****************************************/
  parameter Real Glb_tblGrd[5, 5] = [0, -20, -0.9, -0.89, 40; -20, 0, 0, 0, 0; 1, 0, 0, 0, 0; 2, 0, 0, 0, 0; 40, 0, 0, 0, 0];
  //parameter Real Glb_tblGrd[5, 14] = [0, -30, -5, -0.01, 0, 1.99, 2, 3.99, 4, 5.99, 6, 7.99, 8, 40; -20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 0, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 60, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0];
  /****************************************/
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {66, 15}, extent = {{-60, 0}, {-40, 20}})));
  Visualization.TerrainTableVisualizer VisTerrainTbl(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {46, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerXaxis(shapeType = "box", length = 1.1*(Glb_grdXMax), width = 0.04, height = 0.03, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 61}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerYaxis(shapeType = "box", length = 1.1*(Glb_grdYMax), width = 0.04, height = 0.03, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 39}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerZaxis(shapeType = "box", length = 10*(Glb_grdZMax), width = 0.04, height = 0.03, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder WheelL(r = {0, 0.1, 0}, length = Glb_WheelLength, diameter = Glb_WheelDiameter, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {178, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder FuselageL(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}, w_0_fixed = true, w_0_start = {Glb_w_0_x, Glb_w_0_y, Glb_w_0_z}) annotation(
    Placement(transformation(origin = {202, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenter(m = 0.001, r_0(start = {0, 0, Glb_z0_Ctr}, each fixed = true)) annotation(
    Placement(transformation(origin = {230, 314}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder FuselageR(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}) annotation(
    Placement(transformation(origin = {248, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder WheelR(diameter = Glb_WheelDiameter, length = Glb_WheelLength, r = {0, 0.1, 0}, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {276, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {230, 282}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmL annotation(
    Placement(transformation(origin = {182, 208}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlL(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {198, 174}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdL(animation = false) annotation(
    Placement(transformation(origin = {208, 266}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_L annotation(
    Placement(transformation(origin = {198, 200}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {194, 229}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce forceNgrdR(animation = false) annotation(
    Placement(transformation(origin = {384, 266}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {371, 218}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position positionWhlBtmR annotation(
    Placement(transformation(origin = {352, 207}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGapWhlR(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {368, 170}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensorGrdN_R annotation(
    Placement(transformation(origin = {368, 196}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdR annotation(
    Placement(transformation(origin = {356, 144}, extent = {{-6, -6}, {6, 6}})));
  Components.DiscEdgeBottomTranslation discEdgeBtmL(rDisc = Wheel.diameter/2) annotation(
    Placement(transformation(origin = {152, 300}, extent = {{10, -20}, {-10, 20}})));
  Components.DiscEdgeBottomTranslation discEdgeBtmR(rDisc = Wheel1.diameter/2) annotation(
    Placement(transformation(origin = {302, 300}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmL(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {136, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePositionWhlBtmR(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {290, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelR(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {296, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position positionGrdL annotation(
    Placement(transformation(origin = {186, 142}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_WheelL(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {130, 166}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absoluteL(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {132, 300}, extent = {{5, -5}, {-5, 5}})));
  Components.WheelGroundTangentialForce FtWheelL(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {144, 272}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absoluteR(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {321, 300}, extent = {{-5, -5}, {5, 5}})));
  Components.WheelGroundTangentialForce FtWheelR(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {324, 270}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor_ctr(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world, get_r = true, get_v = true, get_a = true, get_angles = true) annotation(
    Placement(transformation(origin = {217, 321}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles_ctr annotation(
    Placement(transformation(origin = {205, 321}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  //-----------------------------------------------------------------
  parameter Integer nGrdCellX = size(Glb_tblGrd, 1) - 2 "x 方向セル数";
  parameter Integer nGrdCellY = size(Glb_tblGrd, 2) - 2 "y 方向セル数";
  parameter Real Glb_grdZMin = VisTerrainTbl.zMin;
  parameter Real Glb_grdZMax = VisTerrainTbl.zMax;
  parameter Real Glb_grdXMin = Glb_tblGrd[2, 1] "Glb_tblGrd 内の x 最小値";
  parameter Real Glb_grdXMax = Glb_tblGrd[size(Glb_tblGrd, 1), 1] "Glb_tblGrd 内の x 最大値";
  parameter Real Glb_grdYMin = Glb_tblGrd[1, 2] "Glb_tblGrd 内の Y 最小値";
  parameter Real Glb_grdYMax = Glb_tblGrd[1, size(Glb_tblGrd, 2)] "Glb_tblGrd 内の Y 最大値";
  //-----------------------------------------------------------------
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(phi(fixed = false, displayUnit = "deg"), useAxisFlange = false, animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {230, 340}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenterNoRot(m = 0.0, animation = false) annotation(
    Placement(transformation(origin = {244, 358}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles_ctrNoRot annotation(
    Placement(transformation(origin = {219, 363}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor_ctrNoRot(animation = false, get_a = true, get_angles = true, get_r = true, get_v = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {229, 363}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut1(color = {0, 255, 0}, diameter = Glb_StrutDiameter, r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false, length(displayUnit = "m")) annotation(
    Placement(transformation(origin = {80, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket1(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {93, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut1(angle = 1*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {80, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket1(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {80, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {120, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {133, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut(angle = 0*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {120, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {120, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, color = {255, 255, 255}, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {140, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket1(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {100, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut2(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {40, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket2(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {53, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut2(angle = 2*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {40, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket2(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {40, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut3(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {0, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket3(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {13, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut3(angle = 3*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {0, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket3(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {0, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut4(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-40, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket4(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-27, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut4(angle = 4*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-40, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket4(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-40, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket2(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {60, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket3(color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b, animation = Glb_AnimateRocketThrust) annotation(
    Placement(transformation(origin = {20, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket4(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-20, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Gain gain[3](k = {-2, 0, 0}) annotation(
    Placement(transformation(origin = {160, 439}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
  Modelica.Blocks.Math.Add add[3] annotation(
    Placement(transformation(origin = {140, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {137, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add1[3] annotation(
    Placement(transformation(origin = {100, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise1[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {97, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add2[3] annotation(
    Placement(transformation(origin = {60, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {57, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add3[3] annotation(
    Placement(transformation(origin = {20, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise3[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {17, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add4[3] annotation(
    Placement(transformation(origin = {-20, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise4[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-23, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut5(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-80, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket5(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-67, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut5(angle = 5*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-80, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket5(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-80, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket5(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-60, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add5[3] annotation(
    Placement(transformation(origin = {-60, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise5[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-63, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut6(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-120, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket6(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-107, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut6(angle = 6*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-120, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket6(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-120, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket6(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-100, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add6[3] annotation(
    Placement(transformation(origin = {-100, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise6[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-103, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut7(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-160, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket7(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-147, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut7(angle = 7*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-160, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket7(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-160, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket7(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-140, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add7[3] annotation(
    Placement(transformation(origin = {-140, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise7[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-143, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut8(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-200, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket8(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {-187, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut8(angle = 8*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {-200, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket8(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-200, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket8(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-180, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add8[3] annotation(
    Placement(transformation(origin = {-180, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise8[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-183, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut_1_1(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {620, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket_1_1(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {633, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut_1_1(angle = 1*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {620, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket_1_1(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {620, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Strut_1(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {660, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Rocket_1(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {673, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Strut_1(angle = 0*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {660, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_Rocket_1(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {660, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket_1(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {680, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocket_1_1(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {640, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR2(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {580, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR2(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {593, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR2(angle = 2*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {580, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR2(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {580, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR3(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {540, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR3(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {553, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR3(angle = 3*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {540, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR3(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {540, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR4(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {500, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR4(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {513, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR4(angle = 4*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {500, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR4(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {500, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR2(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {600, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR3(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {560, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR4(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {520, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add_1[3] annotation(
    Placement(transformation(origin = {680, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise9[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {677, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add11[3] annotation(
    Placement(transformation(origin = {640, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise11[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {637, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add21[3] annotation(
    Placement(transformation(origin = {600, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise21[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {597, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add31[3] annotation(
    Placement(transformation(origin = {560, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise31[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {557, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Math.Add add41[3] annotation(
    Placement(transformation(origin = {520, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise41[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {517, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR5(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {460, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR5(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {473, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR5(angle = 5*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {460, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR5(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {460, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR5(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {480, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add51[3] annotation(
    Placement(transformation(origin = {480, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise51[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {477, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR6(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {420, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR6(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {433, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR6(angle = 6*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {420, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR6(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {420, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR6(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {440, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add61[3] annotation(
    Placement(transformation(origin = {440, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise61[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {437, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StrutR7(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {380, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR7(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {393, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR7(angle = 7*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {380, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR7(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {380, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR7(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {400, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add71[3] annotation(
    Placement(transformation(origin = {400, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise71[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {397, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder StruR8(color = {0, 255, 0}, diameter = Glb_StrutDiameter, length(displayUnit = "m"), r = {Glb_StrutLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {340, 347}, extent = {{7, -7}, {-7, 7}}, rotation = 270)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder RocketR8(color = {255, 0, 0}, diameter = Glb_RocketDiameter, r = {Glb_RocketLength, 0, 0}, r_0(each fixed = false), w_0_fixed = false) annotation(
    Placement(transformation(origin = {353, 367}, extent = {{7, -7}, {-7, 7}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_StrutR8(angle = 8*Glb_StrutPhaseAngle*180/Modelica.Constants.pi, n = {0, 1, 0}, r(each displayUnit = "m")) annotation(
    Placement(transformation(origin = {340, 332}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRot_RocketR8(angle = -1*Glb_RocketMountAngle*180/Modelica.Constants.pi, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {340, 363}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce fRocketR8(animation = Glb_AnimateRocketThrust, color = {255, 255, 255}, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {360, 385}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add add81[3] annotation(
    Placement(transformation(origin = {360, 400}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const_Noise81[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {357, 413}, extent = {{3, -3}, {-3, 3}}, rotation = 90)));
  Modelica.Blocks.Sources.Ramp ramp_RocketThrustNominal[3](height = {Glb_RocketThrustNominal, 0, 0}, each duration = 1, offset = {0, 0, 0}, startTime = {2, 0, 0}) annotation(
    Placement(transformation(origin = {143, 469}, extent = {{-7, -7}, {7, 7}})));
equation
  connect(FuselageL.frame_b, WheelL.frame_a) annotation(
    Line(points = {{192, 300}, {188, 300}}, color = {95, 95, 95}));
  connect(FuselageR.frame_a, WheelR.frame_b) annotation(
    Line(points = {{258, 300}, {266, 300}}, color = {95, 95, 95}));
  connect(FuselageL.frame_a, FuselageR.frame_b) annotation(
    Line(points = {{212, 300}, {238, 300}}, color = {95, 95, 95}));
  connect(cutForce.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{230, 288}, {230, 310}}, color = {95, 95, 95}));
  connect(FuselageL.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{212, 300}, {212, 305}, {230, 305}, {230, 310}}, color = {95, 95, 95}));
  connect(elastoGapWhlL.flange_b, forceSensorGrdN_L.flange_a) annotation(
    Line(points = {{198, 184}, {198, 195}}, color = {0, 127, 0}));
  connect(positionWhlBtmL.flange, forceSensorGrdN_L.flange_b) annotation(
    Line(points = {{188, 208}, {198, 208}, {198, 205}}, color = {0, 127, 0}));
  connect(const[1].y, forceNgrdL.force[1]) annotation(
    Line(points = {{198, 229}, {208.4, 229}, {208.4, 254}, {208, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const[2].y, forceNgrdL.force[2]) annotation(
    Line(points = {{198, 229}, {208.4, 229}, {208.4, 254}, {208, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceSensorGrdN_L.f, forceNgrdL.force[3]) annotation(
    Line(points = {{203.5, 196}, {208, 196}, {208, 254}}, color = {0, 0, 127}));
  connect(const2[1].y, forceNgrdR.force[1]) annotation(
    Line(points = {{375, 218}, {384.4, 218}, {384.4, 254}, {384, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const2[2].y, forceNgrdR.force[2]) annotation(
    Line(points = {{375, 218}, {384.4, 218}, {384.4, 254}, {384, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGapWhlR.flange_b, forceSensorGrdN_R.flange_a) annotation(
    Line(points = {{368, 180}, {368, 191}}, color = {0, 127, 0}));
  connect(positionWhlBtmR.flange, forceSensorGrdN_R.flange_b) annotation(
    Line(points = {{358, 207}, {368, 207}, {368, 201}}, color = {0, 127, 0}));
  connect(forceSensorGrdN_R.f, forceNgrdR.force[3]) annotation(
    Line(points = {{373.5, 192}, {373.5, 191.5}, {383.5, 191.5}, {383.5, 254}, {384, 254}}, color = {0, 0, 127}));
  connect(positionGrdR.flange, elastoGapWhlR.flange_a) annotation(
    Line(points = {{362, 144}, {368, 144}, {368, 160}}, color = {0, 127, 0}));
  connect(WheelL.frame_b, discEdgeBtmL.frame_a) annotation(
    Line(points = {{168, 300}, {152, 300}}, color = {95, 95, 95}));
  connect(forceNgrdL.frame_b, discEdgeBtmL.frame_btm) annotation(
    Line(points = {{208, 276}, {208, 280}, {152, 280}}, color = {95, 95, 95}));
  connect(WheelR.frame_a, discEdgeBtmR.frame_a) annotation(
    Line(points = {{286, 300}, {302, 300}}, color = {95, 95, 95}));
  connect(forceNgrdR.frame_b, discEdgeBtmR.frame_btm) annotation(
    Line(points = {{384, 276}, {384, 280}, {302, 280}}, color = {95, 95, 95}));
  connect(discEdgeBtmL.frame_btm, absolutePositionWhlBtmL.frame_a) annotation(
    Line(points = {{152, 280}, {152, 254}, {136, 254}, {136, 232}}, color = {95, 95, 95}));
  connect(discEdgeBtmR.frame_btm, absolutePositionWhlBtmR.frame_a) annotation(
    Line(points = {{302, 280}, {302, 254}, {290, 254}, {290, 232}}, color = {95, 95, 95}));
  connect(absolutePositionWhlBtmR.r[1], Table_zGrd_WheelR.u1) annotation(
    Line(points = {{290, 219}, {290, 202.2}, {302, 202.2}, {302, 180}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR.r[2], Table_zGrd_WheelR.u2) annotation(
    Line(points = {{290, 219}, {290, 180}}, color = {0, 0, 127}));
  connect(positionGrdR.s_ref, Table_zGrd_WheelR.y) annotation(
    Line(points = {{348.8, 144}, {295.8, 144}, {295.8, 157}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmL.r[3], positionWhlBtmL.s_ref) annotation(
    Line(points = {{136, 219.4}, {136, 208.4}, {175, 208.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmR.r[3], positionWhlBtmR.s_ref) annotation(
    Line(points = {{290, 219}, {290, 207}, {345, 207}}, color = {0, 0, 127}));
  connect(positionGrdL.flange, elastoGapWhlL.flange_a) annotation(
    Line(points = {{192, 142}, {198, 142}, {198, 164}}, color = {0, 127, 0}));
  connect(absolutePositionWhlBtmL.r[1], Table_zGrd_WheelL.u1) annotation(
    Line(points = {{136, 219.4}, {136, 178.4}}, color = {0, 0, 127}));
  connect(absolutePositionWhlBtmL.r[2], Table_zGrd_WheelL.u2) annotation(
    Line(points = {{136, 219.4}, {136, 214.4}, {124, 214.4}, {124, 178.4}}, color = {0, 0, 127}));
  connect(Table_zGrd_WheelL.y, positionGrdL.s_ref) annotation(
    Line(points = {{130, 155}, {130, 142}, {179, 142}}, color = {0, 0, 127}));
  connect(discEdgeBtmL.frame_a, w_absoluteL.frame_a) annotation(
    Line(points = {{152, 300}, {137, 300}}, color = {95, 95, 95}));
  connect(FtWheelL.frame_b, discEdgeBtmL.frame_btm) annotation(
    Line(points = {{144, 270}, {144, 280}, {152, 280}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_L.f, FtWheelL.u_Fn) annotation(
    Line(points = {{203.5, 196}, {208, 196}, {208, 246}, {144, 246}, {144, 267}}, color = {0, 0, 127}));
  connect(discEdgeBtmR.frame_a, w_absoluteR.frame_a) annotation(
    Line(points = {{302, 300}, {316, 300}}, color = {95, 95, 95}));
  connect(discEdgeBtmR.frame_btm, FtWheelR.frame_b) annotation(
    Line(points = {{302, 280}, {324, 280}, {324, 268}}, color = {95, 95, 95}));
  connect(forceSensorGrdN_R.f, FtWheelR.u_Fn) annotation(
    Line(points = {{373.5, 192}, {384, 192}, {384, 243}, {324, 243}, {324, 265}}, color = {0, 0, 127}));
  connect(bodyCenter.frame_a, absoluteSensor_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {217, 310}, {217, 316}}, color = {95, 95, 95}));
  connect(bodyCenter.frame_a, absoluteAngles_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {205, 310}, {205, 316}}, color = {95, 95, 95}));
  connect(w_absoluteL.w[2], FtWheelL.u_wRoll) annotation(
    Line(points = {{126.5, 300}, {126.5, 288}, {130.5, 288}, {130.5, 276}}, color = {0, 0, 127}));
  connect(w_absoluteR.w[2], FtWheelR.u_wRoll) annotation(
    Line(points = {{326.5, 300}, {326.5, 286}, {310.5, 286}, {310.5, 274}}, color = {0, 0, 127}));
  connect(world.frame_b, markerXaxis.frame_a) annotation(
    Line(points = {{26, 25}, {38, 25}, {38, 61}, {50, 61}}, color = {95, 95, 95}));
  connect(markerYaxis.frame_a, world.frame_b) annotation(
    Line(points = {{50, 40}, {38, 40}, {38, 25}, {26, 25}}, color = {95, 95, 95}));
  connect(world.frame_b, markerZaxis.frame_a) annotation(
    Line(points = {{26, 25}, {38, 25}, {38, 18}, {50, 18}}, color = {95, 95, 95}));
  connect(VisTerrainTbl.frame_a, world.frame_b) annotation(
    Line(points = {{36, 90}, {30, 90}, {30, 25}, {26, 25}}, color = {95, 95, 95}));
  connect(FuselageL.frame_a, revolute.frame_a) annotation(
    Line(points = {{212, 300}, {224, 300}, {224, 340}}, color = {95, 95, 95}));
  connect(revolute.frame_b, bodyCenterNoRot.frame_a) annotation(
    Line(points = {{236, 340}, {244, 340}, {244, 354}}, color = {95, 95, 95}));
  connect(bodyCenterNoRot.frame_a, absoluteSensor_ctrNoRot.frame_a) annotation(
    Line(points = {{244, 354}, {229, 354}, {229, 358}}, color = {95, 95, 95}));
  connect(bodyCenterNoRot.frame_a, absoluteAngles_ctrNoRot.frame_a) annotation(
    Line(points = {{244, 354}, {219, 354}, {219, 358}}, color = {95, 95, 95}));
  connect(fixedRot_Strut1.frame_a, WheelL.frame_b) annotation(
    Line(points = {{80, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut1.frame_b, Strut1.frame_a) annotation(
    Line(points = {{80, 336}, {80, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket1.frame_a, Strut1.frame_b) annotation(
    Line(points = {{80, 359}, {80, 354}}, color = {95, 95, 95}));
  connect(Rocket1.frame_a, fixedRot_Rocket1.frame_b) annotation(
    Line(points = {{86, 367}, {80, 367}}, color = {95, 95, 95}));
  connect(fixedRot_Strut.frame_b, Strut.frame_a) annotation(
    Line(points = {{120, 336}, {120, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket.frame_a, Strut.frame_b) annotation(
    Line(points = {{120, 359}, {120, 354}}, color = {95, 95, 95}));
  connect(Rocket.frame_a, fixedRot_Rocket.frame_b) annotation(
    Line(points = {{126, 367}, {120, 367}}, color = {95, 95, 95}));
  connect(fixedRot_Strut.frame_a, WheelL.frame_b) annotation(
    Line(points = {{120, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fRocket.frame_b, Rocket.frame_b) annotation(
    Line(points = {{140, 380}, {140, 367}}, color = {95, 95, 95}));
  connect(Rocket1.frame_b, fRocket1.frame_b) annotation(
    Line(points = {{100, 368}, {100, 380}}, color = {95, 95, 95}));
  connect(fixedRot_Strut2.frame_b, Strut2.frame_a) annotation(
    Line(points = {{40, 336}, {40, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket2.frame_a, Strut2.frame_b) annotation(
    Line(points = {{40, 359}, {40, 354}}, color = {95, 95, 95}));
  connect(Rocket2.frame_a, fixedRot_Rocket2.frame_b) annotation(
    Line(points = {{46, 367}, {40, 367}}, color = {95, 95, 95}));
  connect(fixedRot_Strut2.frame_a, WheelL.frame_b) annotation(
    Line(points = {{40, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut3.frame_b, Strut3.frame_a) annotation(
    Line(points = {{0, 336}, {0, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket3.frame_a, Strut3.frame_b) annotation(
    Line(points = {{0, 359}, {0, 354}}, color = {95, 95, 95}));
  connect(fixedRot_Strut3.frame_a, WheelL.frame_b) annotation(
    Line(points = {{0, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut4.frame_b, Strut4.frame_a) annotation(
    Line(points = {{-40, 336}, {-40, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket4.frame_a, Strut4.frame_b) annotation(
    Line(points = {{-40, 359}, {-40, 354}}, color = {95, 95, 95}));
  connect(Rocket4.frame_a, fixedRot_Rocket4.frame_b) annotation(
    Line(points = {{-34, 367}, {-40, 367}}, color = {95, 95, 95}));
  connect(fixedRot_Strut4.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-40, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fRocket2.frame_b, Rocket2.frame_b) annotation(
    Line(points = {{60, 380}, {60, 368}}, color = {95, 95, 95}));
  connect(fRocket3.frame_b, Rocket3.frame_b) annotation(
    Line(points = {{20, 380}, {20, 367}}, color = {95, 95, 95}));
  connect(Rocket4.frame_b, fRocket4.frame_b) annotation(
    Line(points = {{-20, 368}, {-20, 380}}, color = {95, 95, 95}));
  connect(Rocket3.frame_a, fixedRot_Rocket3.frame_b) annotation(
    Line(points = {{6, 367}, {0, 367}}, color = {95, 95, 95}));
  connect(add.u1, gain.y) annotation(
    Line(points = {{143, 406}, {143, 436}, {160, 436}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add.u2, const_Noise.y) annotation(
    Line(points = {{137, 406}, {137, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add.y, fRocket.force) annotation(
    Line(points = {{140, 394.5}, {140, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const_Noise1.y, add1.u2) annotation(
    Line(points = {{97, 410}, {97, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add1.y, fRocket1.force) annotation(
    Line(points = {{100, 394.5}, {100, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add1.u1) annotation(
    Line(points = {{160, 436}, {103, 436}, {103, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add2.u2, const_Noise2.y) annotation(
    Line(points = {{57, 406}, {57, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add2.y, fRocket2.force) annotation(
    Line(points = {{60, 394.5}, {60, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add2.u1) annotation(
    Line(points = {{160, 436}, {63, 436}, {63, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add3.u2, const_Noise3.y) annotation(
    Line(points = {{17, 406}, {17, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add3.y, fRocket3.force) annotation(
    Line(points = {{20, 394.5}, {20, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add3.u1) annotation(
    Line(points = {{160, 436}, {23, 436}, {23, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add4.u2, const_Noise4.y) annotation(
    Line(points = {{-23, 406}, {-23, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add4.y, fRocket4.force) annotation(
    Line(points = {{-20, 394.5}, {-20, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add4.u1) annotation(
    Line(points = {{160, 436}, {-17, 436}, {-17, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut5.frame_b, Strut5.frame_a) annotation(
    Line(points = {{-80, 336}, {-80, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket5.frame_a, Strut5.frame_b) annotation(
    Line(points = {{-80, 359}, {-80, 354}}, color = {95, 95, 95}));
  connect(Rocket5.frame_a, fixedRot_Rocket5.frame_b) annotation(
    Line(points = {{-74, 367}, {-80, 367}}, color = {95, 95, 95}));
  connect(fRocket5.frame_b, Rocket5.frame_b) annotation(
    Line(points = {{-60, 380}, {-60, 367}}, color = {95, 95, 95}));
  connect(add5.u2, const_Noise5.y) annotation(
    Line(points = {{-63, 406}, {-63, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add5.y, fRocket5.force) annotation(
    Line(points = {{-60, 394.5}, {-60, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut6.frame_b, Strut6.frame_a) annotation(
    Line(points = {{-120, 336}, {-120, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket6.frame_a, Strut6.frame_b) annotation(
    Line(points = {{-120, 359}, {-120, 354}}, color = {95, 95, 95}));
  connect(Rocket6.frame_a, fixedRot_Rocket6.frame_b) annotation(
    Line(points = {{-114, 367}, {-120, 367}}, color = {95, 95, 95}));
  connect(fRocket6.frame_b, Rocket6.frame_b) annotation(
    Line(points = {{-100, 380}, {-100, 367}}, color = {95, 95, 95}));
  connect(add6.u2, const_Noise6.y) annotation(
    Line(points = {{-103, 406}, {-103, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add6.y, fRocket6.force) annotation(
    Line(points = {{-100, 394.5}, {-100, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut5.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-80, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut6.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-120, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(gain.y, add5.u1) annotation(
    Line(points = {{160, 436}, {-57, 436}, {-57, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add6.u1) annotation(
    Line(points = {{160, 436}, {-97, 436}, {-97, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut7.frame_b, Strut7.frame_a) annotation(
    Line(points = {{-160, 336}, {-160, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket7.frame_a, Strut7.frame_b) annotation(
    Line(points = {{-160, 359}, {-160, 354}}, color = {95, 95, 95}));
  connect(Rocket7.frame_a, fixedRot_Rocket7.frame_b) annotation(
    Line(points = {{-154, 367}, {-160, 367}}, color = {95, 95, 95}));
  connect(fRocket7.frame_b, Rocket7.frame_b) annotation(
    Line(points = {{-140, 380}, {-140, 367}}, color = {95, 95, 95}));
  connect(add7.u2, const_Noise7.y) annotation(
    Line(points = {{-143, 406}, {-143, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add7.y, fRocket7.force) annotation(
    Line(points = {{-140, 394.5}, {-140, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut8.frame_b, Strut8.frame_a) annotation(
    Line(points = {{-200, 336}, {-200, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket8.frame_a, Strut8.frame_b) annotation(
    Line(points = {{-200, 359}, {-200, 354}}, color = {95, 95, 95}));
  connect(Rocket8.frame_a, fixedRot_Rocket8.frame_b) annotation(
    Line(points = {{-194, 367}, {-200, 367}}, color = {95, 95, 95}));
  connect(fRocket8.frame_b, Rocket8.frame_b) annotation(
    Line(points = {{-180, 380}, {-180, 367}}, color = {95, 95, 95}));
  connect(add8.u2, const_Noise8.y) annotation(
    Line(points = {{-183, 406}, {-183, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add8.y, fRocket8.force) annotation(
    Line(points = {{-180, 394.5}, {-180, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut7.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-160, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut8.frame_a, WheelL.frame_b) annotation(
    Line(points = {{-200, 328}, {168, 328}, {168, 300}}, color = {95, 95, 95}));
  connect(gain.y, add7.u1) annotation(
    Line(points = {{160, 436}, {-137, 436}, {-137, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add8.u1) annotation(
    Line(points = {{160, 436}, {-177, 436}, {-177, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut_1_1.frame_b, Strut_1_1.frame_a) annotation(
    Line(points = {{620, 336}, {620, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket_1_1.frame_a, Strut_1_1.frame_b) annotation(
    Line(points = {{620, 359}, {620, 354}}, color = {95, 95, 95}));
  connect(Rocket_1_1.frame_a, fixedRot_Rocket_1_1.frame_b) annotation(
    Line(points = {{626, 367}, {620, 367}}, color = {95, 95, 95}));
  connect(fixedRot_Strut_1.frame_b, Strut_1.frame_a) annotation(
    Line(points = {{660, 336}, {660, 340}}, color = {95, 95, 95}));
  connect(fixedRot_Rocket_1.frame_a, Strut_1.frame_b) annotation(
    Line(points = {{660, 359}, {660, 354}}, color = {95, 95, 95}));
  connect(Rocket_1.frame_a, fixedRot_Rocket_1.frame_b) annotation(
    Line(points = {{666, 367}, {660, 367}}, color = {95, 95, 95}));
  connect(fRocket_1.frame_b, Rocket_1.frame_b) annotation(
    Line(points = {{680, 380}, {680, 367}}, color = {95, 95, 95}));
  connect(Rocket_1_1.frame_b, fRocket_1_1.frame_b) annotation(
    Line(points = {{640, 367}, {640, 379}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR2.frame_b, StrutR2.frame_a) annotation(
    Line(points = {{580, 336}, {580, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR2.frame_a, StrutR2.frame_b) annotation(
    Line(points = {{580, 359}, {580, 354}}, color = {95, 95, 95}));
  connect(RocketR2.frame_a, fixedRot_RocketR2.frame_b) annotation(
    Line(points = {{586, 367}, {580, 367}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR3.frame_b, StrutR3.frame_a) annotation(
    Line(points = {{540, 336}, {540, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR3.frame_a, StrutR3.frame_b) annotation(
    Line(points = {{540, 359}, {540, 354}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR4.frame_b, StrutR4.frame_a) annotation(
    Line(points = {{500, 336}, {500, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR4.frame_a, StrutR4.frame_b) annotation(
    Line(points = {{500, 359}, {500, 354}}, color = {95, 95, 95}));
  connect(RocketR4.frame_a, fixedRot_RocketR4.frame_b) annotation(
    Line(points = {{506, 367}, {500, 367}}, color = {95, 95, 95}));
  connect(fRocketR2.frame_b, RocketR2.frame_b) annotation(
    Line(points = {{600, 380}, {600, 368}}, color = {95, 95, 95}));
  connect(fRocketR3.frame_b, RocketR3.frame_b) annotation(
    Line(points = {{560, 380}, {560, 367}}, color = {95, 95, 95}));
  connect(RocketR4.frame_b, fRocketR4.frame_b) annotation(
    Line(points = {{520, 367}, {520, 379}}, color = {95, 95, 95}));
  connect(RocketR3.frame_a, fixedRot_RocketR3.frame_b) annotation(
    Line(points = {{546, 367}, {540, 367}}, color = {95, 95, 95}));
  connect(add_1.u2, const_Noise9.y) annotation(
    Line(points = {{677, 406}, {677, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add_1.y, fRocket_1.force) annotation(
    Line(points = {{680, 394.5}, {680, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const_Noise11.y, add11.u2) annotation(
    Line(points = {{637, 409.7}, {637, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add11.y, fRocket_1_1.force) annotation(
    Line(points = {{640, 394.5}, {640, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add21.u2, const_Noise21.y) annotation(
    Line(points = {{597, 406}, {597, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add21.y, fRocketR2.force) annotation(
    Line(points = {{600, 394.5}, {600, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add31.u2, const_Noise31.y) annotation(
    Line(points = {{557, 406}, {557, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add31.y, fRocketR3.force) annotation(
    Line(points = {{560, 394.5}, {560, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add41.u2, const_Noise41.y) annotation(
    Line(points = {{517, 406}, {517, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add41.y, fRocketR4.force) annotation(
    Line(points = {{520, 394.5}, {520, 392}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR5.frame_b, StrutR5.frame_a) annotation(
    Line(points = {{460, 336}, {460, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR5.frame_a, StrutR5.frame_b) annotation(
    Line(points = {{460, 359}, {460, 354}}, color = {95, 95, 95}));
  connect(RocketR5.frame_a, fixedRot_RocketR5.frame_b) annotation(
    Line(points = {{466, 367}, {460, 367}}, color = {95, 95, 95}));
  connect(fRocketR5.frame_b, RocketR5.frame_b) annotation(
    Line(points = {{480, 380}, {480, 367}}, color = {95, 95, 95}));
  connect(add51.u2, const_Noise51.y) annotation(
    Line(points = {{477, 406}, {477, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add51.y, fRocketR5.force) annotation(
    Line(points = {{480, 394.5}, {480, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR6.frame_b, StrutR6.frame_a) annotation(
    Line(points = {{420, 336}, {420, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR6.frame_a, StrutR6.frame_b) annotation(
    Line(points = {{420, 359}, {420, 354}}, color = {95, 95, 95}));
  connect(RocketR6.frame_a, fixedRot_RocketR6.frame_b) annotation(
    Line(points = {{426, 367}, {420, 367}}, color = {95, 95, 95}));
  connect(fRocketR6.frame_b, RocketR6.frame_b) annotation(
    Line(points = {{440, 380}, {440, 367}}, color = {95, 95, 95}));
  connect(add61.u2, const_Noise61.y) annotation(
    Line(points = {{437, 406}, {437, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add61.y, fRocketR6.force) annotation(
    Line(points = {{440, 394.5}, {440, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR7.frame_b, StrutR7.frame_a) annotation(
    Line(points = {{380, 336}, {380, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR7.frame_a, StrutR7.frame_b) annotation(
    Line(points = {{380, 359}, {380, 354}}, color = {95, 95, 95}));
  connect(RocketR7.frame_a, fixedRot_RocketR7.frame_b) annotation(
    Line(points = {{386, 367}, {380, 367}}, color = {95, 95, 95}));
  connect(fRocketR7.frame_b, RocketR7.frame_b) annotation(
    Line(points = {{400, 380}, {400, 367}}, color = {95, 95, 95}));
  connect(add71.u2, const_Noise71.y) annotation(
    Line(points = {{397, 406}, {397, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add71.y, fRocketR7.force) annotation(
    Line(points = {{400, 394.5}, {400, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR8.frame_b, StruR8.frame_a) annotation(
    Line(points = {{340, 336}, {340, 340}}, color = {95, 95, 95}));
  connect(fixedRot_RocketR8.frame_a, StruR8.frame_b) annotation(
    Line(points = {{340, 359}, {340, 354}}, color = {95, 95, 95}));
  connect(RocketR8.frame_a, fixedRot_RocketR8.frame_b) annotation(
    Line(points = {{346, 367}, {340, 367}}, color = {95, 95, 95}));
  connect(fRocketR8.frame_b, RocketR8.frame_b) annotation(
    Line(points = {{360, 380}, {360, 367}}, color = {95, 95, 95}));
  connect(add81.u2, const_Noise81.y) annotation(
    Line(points = {{357, 406}, {357, 410}}, color = {0, 0, 127}, thickness = 0.5));
  connect(add81.y, fRocketR8.force) annotation(
    Line(points = {{360, 394.5}, {360, 392.5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_StrutR8.frame_a, WheelR.frame_a) annotation(
    Line(points = {{340, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR7.frame_a, WheelR.frame_a) annotation(
    Line(points = {{380, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR6.frame_a, WheelR.frame_a) annotation(
    Line(points = {{420, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR5.frame_a, WheelR.frame_a) annotation(
    Line(points = {{460, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR4.frame_a, WheelR.frame_a) annotation(
    Line(points = {{500, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR3.frame_a, WheelR.frame_a) annotation(
    Line(points = {{540, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_StrutR2.frame_a, WheelR.frame_a) annotation(
    Line(points = {{580, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(fixedRot_Strut_1_1.frame_a, WheelR.frame_a) annotation(
    Line(points = {{620, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(gain.y, add81.u1) annotation(
    Line(points = {{160, 436}, {363, 436}, {363, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add71.u1) annotation(
    Line(points = {{160, 436}, {403, 436}, {403, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add61.u1) annotation(
    Line(points = {{160, 436}, {443, 436}, {443, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add51.u1) annotation(
    Line(points = {{160, 436}, {483, 436}, {483, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add41.u1) annotation(
    Line(points = {{160, 436}, {523, 436}, {523, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add31.u1) annotation(
    Line(points = {{160, 436}, {563, 436}, {563, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add21.u1) annotation(
    Line(points = {{160, 436}, {603, 436}, {603, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(fixedRot_Strut_1.frame_a, WheelR.frame_a) annotation(
    Line(points = {{660, 328}, {286, 328}, {286, 300}}, color = {95, 95, 95}));
  connect(gain.y, add11.u1) annotation(
    Line(points = {{160, 436}, {643, 436}, {643, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(gain.y, add_1.u1) annotation(
    Line(points = {{160, 436}, {683, 436}, {683, 406}}, color = {0, 0, 127}, thickness = 0.5));
  connect(ramp_RocketThrustNominal.y, gain.u) annotation(
    Line(points = {{151, 469}, {160, 469}, {160, 442}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Diagram(coordinateSystem(extent = {{-220, 480}, {700, 0}}), graphics = {Text(origin = {170, 449}, extent = {{-46, 5}, {46, -5}}, textString = "multiply by 2 because each strut has 2 rocket motors", horizontalAlignment = TextAlignment.Left)}),
    version = "",
    uses(Modelica(version = "4.1.0")),
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.05),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end ProtoPanjan_006;