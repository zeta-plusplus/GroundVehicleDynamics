within GroundVehicleDynamics.Examples.Projects.Panjan;

model ProtoPanjan_005
  extends Modelica.Icons.Example;
  /****************************************/
  parameter Modelica.Units.SI.TranslationalSpringConstant Glb_c_GrdCntct = 1e7;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_d_GrdCntct = 1e6;
  parameter Modelica.Units.SI.Length Glb_s_rel0_GrdCntct = 1e-5;
  parameter Modelica.Units.SI.Length Glb_x0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_y0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_z0_Ctr = Glb_WheelDiameter/2 + Glb_grdZMax + 0.1;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_x = 0;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_y = 10;
  parameter Modelica.Units.SI.AngularVelocity Glb_w_0_z = 0;
  parameter Modelica.Units.SI.Length Glb_WheelDiameter = 3;
  parameter Modelica.Units.SI.Length Glb_WheelLength = 0.1;
  parameter Modelica.Units.SI.Length Glb_FuselageDiameter = 1;
  parameter Modelica.Units.SI.Length Glb_FuselageLength = 1;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSlip = 1000;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_WheelcSide = 5000;
  /****************************************/
  //parameter Real Glb_tblGrd[5, 5] = [0, -20, -0.9, -0.89, 40; -20, 0, 0, 0, 0; 1, 0, 0, 0, 0; 2, 0, 0, 0, 0; 40, 0, 0, 0, 0];
  parameter Real Glb_tblGrd[5, 14] = [0, -30, -5, -0.01, 0, 1.99, 2, 3.99, 4, 5.99, 6, 7.99, 8, 40; -20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 0, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 60, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0];
  /****************************************/
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {66, 15}, extent = {{-60, 0}, {-40, 20}})));
  GroundVehicleDynamics.Visualization.TerrainTableVisualizer VisTerrainTbl(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {46, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerXaxis(shapeType = "box", length = 1.1*(Glb_grdXMax), width = 0.04, height = 0.03, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 61}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerYaxis(shapeType = "box", length = 1.1*(Glb_grdYMax), width = 0.04, height = 0.03, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 39}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerZaxis(shapeType = "box", length = 10*(Glb_grdZMax), width = 0.04, height = 0.03, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel(r = {0, 0.1, 0}, length = Glb_WheelLength, diameter = Glb_WheelDiameter, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {176, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}, w_0_fixed = true, w_0_start = {Glb_w_0_x, Glb_w_0_y, Glb_w_0_z}) annotation(
    Placement(transformation(origin = {202, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenter(m = 0.001, r_0(start = {0, 0, Glb_z0_Ctr}, each fixed = true)) annotation(
    Placement(transformation(origin = {230, 314}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage1(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}) annotation(
    Placement(transformation(origin = {248, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel1(diameter = Glb_WheelDiameter, length = Glb_WheelLength, r = {0, 0.1, 0}, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {274, 300}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {230, 282}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
    Placement(transformation(origin = {182, 208}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {198, 174}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(animation = false) annotation(
    Placement(transformation(origin = {208, 266}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor annotation(
    Placement(transformation(origin = {198, 200}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {194, 229}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force1(animation = false) annotation(
    Placement(transformation(origin = {384, 266}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {371, 218}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position position1 annotation(
    Placement(transformation(origin = {352, 207}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap1(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {368, 170}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor1 annotation(
    Placement(transformation(origin = {368, 196}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position position_2 annotation(
    Placement(transformation(origin = {356, 144}, extent = {{-6, -6}, {6, 6}})));
  Components.DiscEdgeBottomTranslation discEdgeBtm(rDisc = Wheel.diameter/2) annotation(
    Placement(transformation(origin = {152, 300}, extent = {{10, -20}, {-10, 20}})));
  Components.DiscEdgeBottomTranslation discEdgeBtm1(rDisc = Wheel1.diameter/2) annotation(
    Placement(transformation(origin = {302, 300}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {136, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {290, 226}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel1(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {296, 168}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_0deg(animation = true, r = {cos(0*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(0*Modelica.Constants.pi/180)*Wheel.diameter/2}, color = {255, 0, 0}) annotation(
    Placement(transformation(origin = {128, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_40deg(animation = true, color = {255, 0, 0}, r = {cos(40*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(40*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {110, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_80deg(animation = true, color = {255, 0, 0}, r = {cos(80*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(80*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {94, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_120deg(animation = true, color = {255, 0, 0}, r = {cos(120*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(120*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {76, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_160deg(animation = true, color = {255, 0, 0}, r = {cos(160*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(160*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {58, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_200deg(animation = true, color = {255, 0, 0}, r = {cos(200*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(200*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {42, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_240deg(animation = true, color = {255, 0, 0}, r = {cos(240*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(240*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {24, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_280deg(animation = true, color = {255, 0, 0}, r = {cos(280*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(280*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {4, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_320deg(animation = true, color = {255, 0, 0}, r = {cos(320*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(320*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {-16, 322}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_0deg(animation = true, color = {255, 0, 0}, r = {cos(0*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(0*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {476, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_40deg(animation = true, color = {255, 0, 0}, r = {cos(40*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(40*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {458, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_80deg(animation = true, color = {255, 0, 0}, r = {cos(80*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(80*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {442, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_120deg(animation = true, color = {255, 0, 0}, r = {cos(120*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(120*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {424, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_160deg(animation = true, color = {255, 0, 0}, r = {cos(160*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(160*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {406, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_200deg(animation = true, color = {255, 0, 0}, r = {cos(200*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(200*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {390, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_240deg(animation = true, color = {255, 0, 0}, r = {cos(240*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(240*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {372, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_280deg(animation = true, color = {255, 0, 0}, r = {cos(280*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(280*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {352, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_320deg(animation = true, color = {255, 0, 0}, r = {cos(320*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(320*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {332, 320}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position position2 annotation(
    Placement(transformation(origin = {186, 142}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {130, 166}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absolute(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {132, 300}, extent = {{5, -5}, {-5, 5}})));
  Components.WheelGroundTangentialForce FtWheel(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {144, 272}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absolute1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {321, 300}, extent = {{-5, -5}, {5, 5}})));
  Components.WheelGroundTangentialForce FtWheel1(rDisc = Glb_WheelDiameter/2) annotation(
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
equation
  connect(Fuselage.frame_b, Wheel.frame_a) annotation(
    Line(points = {{192, 300}, {186, 300}}, color = {95, 95, 95}));
  connect(Fuselage1.frame_a, Wheel1.frame_b) annotation(
    Line(points = {{258, 300}, {264, 300}}, color = {95, 95, 95}));
  connect(Fuselage.frame_a, Fuselage1.frame_b) annotation(
    Line(points = {{212, 300}, {238, 300}}, color = {95, 95, 95}));
  connect(cutForce.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{230, 288}, {230, 310}}, color = {95, 95, 95}));
  connect(Fuselage.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{212, 300}, {212, 305}, {230, 305}, {230, 310}}, color = {95, 95, 95}));
  connect(elastoGap.flange_b, forceSensor.flange_a) annotation(
    Line(points = {{198, 184}, {198, 195}}, color = {0, 127, 0}));
  connect(position.flange, forceSensor.flange_b) annotation(
    Line(points = {{188, 208}, {198, 208}, {198, 205}}, color = {0, 127, 0}));
  connect(const[1].y, force.force[1]) annotation(
    Line(points = {{198, 229}, {208.4, 229}, {208.4, 254}, {208, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const[2].y, force.force[2]) annotation(
    Line(points = {{198, 229}, {208.4, 229}, {208.4, 254}, {208, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceSensor.f, force.force[3]) annotation(
    Line(points = {{203.5, 196}, {208, 196}, {208, 254}}, color = {0, 0, 127}));
  connect(const2[1].y, force1.force[1]) annotation(
    Line(points = {{375, 218}, {384.4, 218}, {384.4, 254}, {384, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const2[2].y, force1.force[2]) annotation(
    Line(points = {{375, 218}, {384.4, 218}, {384.4, 254}, {384, 254}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGap1.flange_b, forceSensor1.flange_a) annotation(
    Line(points = {{368, 180}, {368, 191}}, color = {0, 127, 0}));
  connect(position1.flange, forceSensor1.flange_b) annotation(
    Line(points = {{358, 207}, {368, 207}, {368, 201}}, color = {0, 127, 0}));
  connect(forceSensor1.f, force1.force[3]) annotation(
    Line(points = {{373.5, 192}, {373.5, 191.5}, {383.5, 191.5}, {383.5, 254}, {384, 254}}, color = {0, 0, 127}));
  connect(position_2.flange, elastoGap1.flange_a) annotation(
    Line(points = {{362, 144}, {368, 144}, {368, 160}}, color = {0, 127, 0}));
  connect(Wheel.frame_b, discEdgeBtm.frame_a) annotation(
    Line(points = {{166, 300}, {152, 300}}, color = {95, 95, 95}));
  connect(force.frame_b, discEdgeBtm.frame_btm) annotation(
    Line(points = {{208, 276}, {208, 280}, {152, 280}}, color = {95, 95, 95}));
  connect(Wheel1.frame_a, discEdgeBtm1.frame_a) annotation(
    Line(points = {{284, 300}, {302, 300}}, color = {95, 95, 95}));
  connect(force1.frame_b, discEdgeBtm1.frame_btm) annotation(
    Line(points = {{384, 276}, {384, 280}, {302, 280}}, color = {95, 95, 95}));
  connect(discEdgeBtm.frame_btm, absolutePosition.frame_a) annotation(
    Line(points = {{152, 280}, {152, 254}, {136, 254}, {136, 232}}, color = {95, 95, 95}));
  connect(discEdgeBtm1.frame_btm, absolutePosition1.frame_a) annotation(
    Line(points = {{302, 280}, {302, 254}, {290, 254}, {290, 232}}, color = {95, 95, 95}));
  connect(absolutePosition1.r[1], Table_zGrd_Wheel1.u1) annotation(
    Line(points = {{290, 219}, {290, 202.2}, {302, 202.2}, {302, 180}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[2], Table_zGrd_Wheel1.u2) annotation(
    Line(points = {{290, 219}, {290, 180}}, color = {0, 0, 127}));
  connect(position_2.s_ref, Table_zGrd_Wheel1.y) annotation(
    Line(points = {{348.8, 144}, {295.8, 144}, {295.8, 157}}, color = {0, 0, 127}));
  connect(Wheel.frame_b, strut_0deg.frame_a) annotation(
    Line(points = {{166, 300}, {166, 312}, {128, 312}}, color = {95, 95, 95}));
  connect(strut_40deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{110, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_80deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{94, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_120deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{76, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(absolutePosition.r[3], position.s_ref) annotation(
    Line(points = {{136, 219.4}, {136, 208.4}, {175, 208.4}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[3], position1.s_ref) annotation(
    Line(points = {{290, 219}, {290, 207}, {345, 207}}, color = {0, 0, 127}));
  connect(strut_160deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{58, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_200deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{42, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_240deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{24, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_280deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{4, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_320deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{-16, 312}, {166, 312}, {166, 300}}, color = {95, 95, 95}));
  connect(strut_1_320deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{332, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_280deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{352, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_240deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{372, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_200deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{390, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_160deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{406, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_120deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{424, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_80deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{442, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_40deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{458, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(strut_1_0deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{476, 310}, {284, 310}, {284, 300}}, color = {95, 95, 95}));
  connect(position2.flange, elastoGap.flange_a) annotation(
    Line(points = {{192, 142}, {198, 142}, {198, 164}}, color = {0, 127, 0}));
  connect(absolutePosition.r[1], Table_zGrd_Wheel.u1) annotation(
    Line(points = {{136, 219.4}, {136, 178.4}}, color = {0, 0, 127}));
  connect(absolutePosition.r[2], Table_zGrd_Wheel.u2) annotation(
    Line(points = {{136, 219.4}, {136, 214.4}, {124, 214.4}, {124, 178.4}}, color = {0, 0, 127}));
  connect(Table_zGrd_Wheel.y, position2.s_ref) annotation(
    Line(points = {{130, 155}, {130, 142}, {179, 142}}, color = {0, 0, 127}));
  connect(discEdgeBtm.frame_a, w_absolute.frame_a) annotation(
    Line(points = {{152, 300}, {137, 300}}, color = {95, 95, 95}));
  connect(FtWheel.frame_b, discEdgeBtm.frame_btm) annotation(
    Line(points = {{144, 270}, {144, 280}, {152, 280}}, color = {95, 95, 95}));
  connect(forceSensor.f, FtWheel.u_Fn) annotation(
    Line(points = {{203.5, 196}, {208, 196}, {208, 246}, {144, 246}, {144, 267}}, color = {0, 0, 127}));
  connect(discEdgeBtm1.frame_a, w_absolute1.frame_a) annotation(
    Line(points = {{302, 300}, {316, 300}}, color = {95, 95, 95}));
  connect(discEdgeBtm1.frame_btm, FtWheel1.frame_b) annotation(
    Line(points = {{302, 280}, {324, 280}, {324, 268}}, color = {95, 95, 95}));
  connect(forceSensor1.f, FtWheel1.u_Fn) annotation(
    Line(points = {{373.5, 192}, {384, 192}, {384, 243}, {324, 243}, {324, 265}}, color = {0, 0, 127}));
  connect(bodyCenter.frame_a, absoluteSensor_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {217, 310}, {217, 316}}, color = {95, 95, 95}));
  connect(bodyCenter.frame_a, absoluteAngles_ctr.frame_a) annotation(
    Line(points = {{230, 310}, {205, 310}, {205, 316}}, color = {95, 95, 95}));
  connect(w_absolute.w[2], FtWheel.u_wRoll) annotation(
    Line(points = {{126.5, 300}, {126.5, 288}, {130.5, 288}, {130.5, 276}}, color = {0, 0, 127}));
  connect(w_absolute1.w[2], FtWheel1.u_wRoll) annotation(
    Line(points = {{326.5, 300}, {326.5, 286}, {310.5, 286}, {310.5, 274}}, color = {0, 0, 127}));
  connect(world.frame_b, markerXaxis.frame_a) annotation(
    Line(points = {{26, 25}, {38, 25}, {38, 61}, {50, 61}}, color = {95, 95, 95}));
  connect(markerYaxis.frame_a, world.frame_b) annotation(
    Line(points = {{50, 40}, {38, 40}, {38, 25}, {26, 25}}, color = {95, 95, 95}));
  connect(world.frame_b, markerZaxis.frame_a) annotation(
    Line(points = {{26, 25}, {38, 25}, {38, 18}, {50, 18}}, color = {95, 95, 95}));
  connect(VisTerrainTbl.frame_a, world.frame_b) annotation(
    Line(points = {{36, 90}, {30, 90}, {30, 25}, {26, 25}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-40, 340}, {500, 0}})),
    version = "",
    uses(Modelica(version = "4.1.0")),
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-09, Interval = 0.02),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end ProtoPanjan_005;