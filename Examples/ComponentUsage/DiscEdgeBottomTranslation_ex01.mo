within GroundVehicleDynamics.Examples.ComponentUsage;

model DiscEdgeBottomTranslation_ex01
  extends Modelica.Icons.Example;
  /****************************************/
  parameter Modelica.Units.SI.TranslationalSpringConstant Glb_c_GrdCntct = 1e7;
  parameter Modelica.Units.SI.TranslationalDampingConstant Glb_d_GrdCntct = 1e6;
  parameter Modelica.Units.SI.Length Glb_s_rel0_GrdCntct = 1e-5;
  parameter Modelica.Units.SI.Length Glb_x0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_y0_Ctr = 0;
  parameter Modelica.Units.SI.Length Glb_z0_Ctr = 2;
  /****************************************/
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {66, 2}, extent = {{-60, 0}, {-40, 20}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel(r = {0, 0.1, 0}, length = 0.1, diameter = 3, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {176, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage(diameter = 1, length = 1, r = {0, 1, 0}, w_0_fixed = true, w_0_start = {0, 1, 0}) annotation(
    Placement(transformation(origin = {202, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenter(m = 0.001, r_0(start = {0, 0, Glb_z0_Ctr}, each fixed = true)) annotation(
    Placement(transformation(origin = {230, 234}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage1(diameter = 1, length = 1, r = {0, 1, 0}) annotation(
    Placement(transformation(origin = {248, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel1(diameter = 3, length = 0.1, r = {0, 0.1, 0}, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {274, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {230, 202}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
    Placement(transformation(origin = {182, 135}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {198, 106}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(animation = false) annotation(
    Placement(transformation(origin = {208, 166}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor annotation(
    Placement(transformation(origin = {198, 125}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {198, 148}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force1(animation = false) annotation(
    Placement(transformation(origin = {384, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {374, 142}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position position1 annotation(
    Placement(transformation(origin = {352, 129}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap1(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {368, 100}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor1 annotation(
    Placement(transformation(origin = {368, 119}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position position_2 annotation(
    Placement(transformation(origin = {356, 74}, extent = {{-6, -6}, {6, 6}})));
  Components.DiscEdgeBottomTranslation discEdgeBtm(rDisc = Wheel.diameter/2) annotation(
    Placement(transformation(origin = {152, 220}, extent = {{10, -20}, {-10, 20}})));
  Components.DiscEdgeBottomTranslation discEdgeBtm1(rDisc = Wheel1.diameter/2) annotation(
    Placement(transformation(origin = {302, 220}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {152, 184}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {302, 184}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel1(table = [0, -1.1, -0.9, -0.89, -0.8; 0, 0.2, 0.2, 0, 0; 1, 0.2, 0.2, 0, 0; 2, 0.2, 0.2, 0, 0; 10, 0.2, 0.2, 0, 0]) annotation(
    Placement(transformation(origin = {296, 98}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape visObs(length = 10, width = 5, height = 0.2, color = {0, 255, 0}, r_shape = {-visObs.length/2, -visObs.width/2 - 0.9, visObs.height/2}) annotation(
    Placement(transformation(origin = {60, 12}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_0deg(animation = true, r = {cos(0*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(0*Modelica.Constants.pi/180)*Wheel.diameter/2}, color = {255, 0, 0}) annotation(
    Placement(transformation(origin = {128, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_40deg(animation = true, color = {255, 0, 0}, r = {cos(40*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(40*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {110, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_80deg(animation = true, color = {255, 0, 0}, r = {cos(80*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(80*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {94, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_120deg(animation = true, color = {255, 0, 0}, r = {cos(120*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(120*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {76, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape visGrd(color = {0, 0, 0}, height = 0, length = 20, width = 20, animation = false) annotation(
    Placement(transformation(origin = {46, 34}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_160deg(animation = true, color = {255, 0, 0}, r = {cos(160*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(160*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {58, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_200deg(animation = true, color = {255, 0, 0}, r = {cos(200*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(200*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {42, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_240deg(animation = true, color = {255, 0, 0}, r = {cos(240*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(240*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {24, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_280deg(animation = true, color = {255, 0, 0}, r = {cos(280*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(280*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {4, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_320deg(animation = true, color = {255, 0, 0}, r = {cos(320*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(320*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {-16, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_0deg(animation = true, color = {255, 0, 0}, r = {cos(0*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(0*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {476, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_40deg(animation = true, color = {255, 0, 0}, r = {cos(40*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(40*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {458, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_80deg(animation = true, color = {255, 0, 0}, r = {cos(80*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(80*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {442, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_120deg(animation = true, color = {255, 0, 0}, r = {cos(120*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(120*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {424, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_160deg(animation = true, color = {255, 0, 0}, r = {cos(160*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(160*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {406, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_200deg(animation = true, color = {255, 0, 0}, r = {cos(200*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(200*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {390, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_240deg(animation = true, color = {255, 0, 0}, r = {cos(240*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(240*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {372, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_280deg(animation = true, color = {255, 0, 0}, r = {cos(280*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(280*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {352, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_1_320deg(animation = true, color = {255, 0, 0}, r = {cos(320*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(320*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {332, 240}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position position2 annotation(
    Placement(transformation(origin = {186, 74}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel(table = [0, -1.1, -0.9, -0.89, -0.8; 0, 0.2, 0.2, 0, 0; 1, 0.2, 0.2, 0, 0; 2, 0.2, 0.2, 0, 0; 10, 0.2, 0.2, 0, 0]) annotation(
    Placement(transformation(origin = {130, 98}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(Fuselage.frame_b, Wheel.frame_a) annotation(
    Line(points = {{192, 220}, {186, 220}}, color = {95, 95, 95}));
  connect(Fuselage1.frame_a, Wheel1.frame_b) annotation(
    Line(points = {{258, 220}, {264, 220}}, color = {95, 95, 95}));
  connect(Fuselage.frame_a, Fuselage1.frame_b) annotation(
    Line(points = {{212, 220}, {238, 220}}, color = {95, 95, 95}));
  connect(cutForce.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{230, 208}, {230, 230}}, color = {95, 95, 95}));
  connect(Fuselage.frame_a, bodyCenter.frame_a) annotation(
    Line(points = {{212, 220}, {212, 225}, {230, 225}, {230, 230}}, color = {95, 95, 95}));
  connect(elastoGap.flange_b, forceSensor.flange_a) annotation(
    Line(points = {{198, 116}, {198, 120}}, color = {0, 127, 0}));
  connect(position.flange, forceSensor.flange_b) annotation(
    Line(points = {{188, 135}, {198, 135}, {198, 130}}, color = {0, 127, 0}));
  connect(const[1].y, force.force[1]) annotation(
    Line(points = {{202.4, 148}, {208.4, 148}, {208.4, 154}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const[2].y, force.force[2]) annotation(
    Line(points = {{202.4, 148}, {208.4, 148}, {208.4, 154}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceSensor.f, force.force[3]) annotation(
    Line(points = {{203.5, 121}, {208, 121}, {208, 154}}, color = {0, 0, 127}));
  connect(const2[1].y, force1.force[1]) annotation(
    Line(points = {{378.4, 142}, {384.4, 142}, {384.4, 148}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const2[2].y, force1.force[2]) annotation(
    Line(points = {{378.4, 142}, {384.4, 142}, {384.4, 148}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGap1.flange_b, forceSensor1.flange_a) annotation(
    Line(points = {{368, 110}, {368, 114}}, color = {0, 127, 0}));
  connect(position1.flange, forceSensor1.flange_b) annotation(
    Line(points = {{358, 129}, {368, 129}, {368, 124}}, color = {0, 127, 0}));
  connect(forceSensor1.f, force1.force[3]) annotation(
    Line(points = {{373.5, 115}, {383.5, 115}, {383.5, 147}}, color = {0, 0, 127}));
  connect(position_2.flange, elastoGap1.flange_a) annotation(
    Line(points = {{362, 74}, {368, 74}, {368, 90}}, color = {0, 127, 0}));
  connect(Wheel.frame_b, discEdgeBtm.frame_a) annotation(
    Line(points = {{166, 220}, {152, 220}}, color = {95, 95, 95}));
  connect(force.frame_b, discEdgeBtm.frame_btm) annotation(
    Line(points = {{208, 176}, {208, 200}, {152, 200}}, color = {95, 95, 95}));
  connect(Wheel1.frame_a, discEdgeBtm1.frame_a) annotation(
    Line(points = {{284, 220}, {302, 220}}, color = {95, 95, 95}));
  connect(force1.frame_b, discEdgeBtm1.frame_btm) annotation(
    Line(points = {{384, 170}, {384, 200}, {302, 200}}, color = {95, 95, 95}));
  connect(discEdgeBtm.frame_btm, absolutePosition.frame_a) annotation(
    Line(points = {{152, 200}, {152, 190}}, color = {95, 95, 95}));
  connect(discEdgeBtm1.frame_btm, absolutePosition1.frame_a) annotation(
    Line(points = {{302, 200}, {302, 190}}, color = {95, 95, 95}));
  connect(absolutePosition1.r[1], Table_zGrd_Wheel1.u1) annotation(
    Line(points = {{302, 177.4}, {302, 109.4}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[2], Table_zGrd_Wheel1.u2) annotation(
    Line(points = {{302, 177.4}, {302, 151.4}, {290, 151.4}, {290, 109.4}}, color = {0, 0, 127}));
  connect(position_2.s_ref, Table_zGrd_Wheel1.y) annotation(
    Line(points = {{348.8, 74}, {296.8, 74}, {296.8, 88}}, color = {0, 0, 127}));
  connect(Wheel.frame_b, strut_0deg.frame_a) annotation(
    Line(points = {{166, 220}, {166, 232}, {128, 232}}, color = {95, 95, 95}));
  connect(strut_40deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{110, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_80deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{94, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_120deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{76, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(absolutePosition.r[3], position.s_ref) annotation(
    Line(points = {{152, 177.4}, {152, 134.4}, {175, 134.4}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[3], position1.s_ref) annotation(
    Line(points = {{302, 177.4}, {302, 128.4}, {345, 128.4}}, color = {0, 0, 127}));
  connect(world.frame_b, visGrd.frame_a) annotation(
    Line(points = {{26, 12}, {36, 12}, {36, 34}}, color = {95, 95, 95}));
  connect(world.frame_b, visObs.frame_a) annotation(
    Line(points = {{26, 12}, {50, 12}}, color = {95, 95, 95}));
  connect(strut_160deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{58, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_200deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{42, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_240deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{24, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_280deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{4, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_320deg.frame_a, Wheel.frame_b) annotation(
    Line(points = {{-16, 232}, {166, 232}, {166, 220}}, color = {95, 95, 95}));
  connect(strut_1_320deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{332, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_280deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{352, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_240deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{372, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_200deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{390, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_160deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{406, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_120deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{424, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_80deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{442, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_40deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{458, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(strut_1_0deg.frame_a, Wheel1.frame_a) annotation(
    Line(points = {{476, 230}, {284, 230}, {284, 220}}, color = {95, 95, 95}));
  connect(position2.flange, elastoGap.flange_a) annotation(
    Line(points = {{192, 74}, {198, 74}, {198, 96}}, color = {0, 127, 0}));
  connect(absolutePosition.r[1], Table_zGrd_Wheel.u1) annotation(
    Line(points = {{152, 178}, {152, 136}, {136, 136}, {136, 110}}, color = {0, 0, 127}));
  connect(absolutePosition.r[2], Table_zGrd_Wheel.u2) annotation(
    Line(points = {{152, 178}, {152, 142}, {124, 142}, {124, 110}}, color = {0, 0, 127}));
  connect(Table_zGrd_Wheel.y, position2.s_ref) annotation(
    Line(points = {{130, 88}, {130, 74}, {179, 74}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-40, 260}, {500, 0}})),
    version = "",
    uses(Modelica(version = "4.1.0")),
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-09, Interval = 0.01),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end DiscEdgeBottomTranslation_ex01;
