within GroundVehicleDynamics.Examples.Projects.Panjan;

model ProtoPanjan_004
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
  //parameter Real Glb_tblGrd[5,5]=[0, -10, -0.9, -0.89, 0; 0, 0, 0, 0, 0; 1, 0, 0, 0, 0; 2, 0, 0, 0, 0; 10, 0, 0, 0, 0];
  //parameter Real Glb_tblGrd[5,8]=[0, -10, -5, -0.02, -0.01, 0, 5, 10; -2, 0.2, 0.2, 0.2, 0.1, 0, 0, 0; 0, 0.2, 0.2, 0.2, 0.1, 0, 0, 0; 2, 0.2, 0.2, 0.2, 0.1, 0, 0, 0; 10, 0.2, 0.2, 0.2, 0.1, 0, 0, 0];
  //parameter Real Glb_tblGrd[5,10]=[0, -30, -5, -0.01, 0, 4.99, 5, 9.99, 10, 40; -20, 0.6, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 0, 0.6, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 20, 0.6, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 60, 0.6, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0];
  parameter Real Glb_tblGrd[5, 14] = [0, -30, -5, -0.01, 0, 1.99, 2, 3.99, 4, 5.99, 6, 7.99, 8, 40; -20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 0, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 20, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0; 60, 1, 1, 1, 0.8, 0.8, 0.6, 0.6, 0.4, 0.4, 0.2, 0.2, 0, 0];
  /****************************************/
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {66, 11}, extent = {{-60, 0}, {-40, 20}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel(r = {0, 0.1, 0}, length = Glb_WheelLength, diameter = Glb_WheelDiameter, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {176, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}, w_0_fixed = true, w_0_start = {Glb_w_0_x, Glb_w_0_y, Glb_w_0_z}) annotation(
    Placement(transformation(origin = {202, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyCenter(m = 0.001, r_0(start = {0, 0, Glb_z0_Ctr}, each fixed = true)) annotation(
    Placement(transformation(origin = {230, 234}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Fuselage1(diameter = Glb_FuselageDiameter, length = Glb_FuselageLength, r = {0, 1, 0}) annotation(
    Placement(transformation(origin = {248, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder Wheel1(diameter = Glb_WheelDiameter, length = Glb_WheelLength, r = {0, 0.1, 0}, r_0(each fixed = false), w_0_fixed = false, w_0_start = {0, 10, 0}) annotation(
    Placement(transformation(origin = {274, 220}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {230, 202}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
    Placement(transformation(origin = {182, 134}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {198, 106}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(animation = false) annotation(
    Placement(transformation(origin = {208, 186}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor annotation(
    Placement(transformation(origin = {198, 126}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {196, 145}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force1(animation = false) annotation(
    Placement(transformation(origin = {384, 186}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const2[3](k = {0, 0, 0}) annotation(
    Placement(transformation(origin = {371, 140}, extent = {{-4, -4}, {4, 4}})));
  Modelica.Mechanics.Translational.Sources.Position position1 annotation(
    Placement(transformation(origin = {352, 129}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Translational.Components.ElastoGap elastoGap1(c = Glb_c_GrdCntct, d = Glb_d_GrdCntct, s_rel0 = Glb_s_rel0_GrdCntct) annotation(
    Placement(transformation(origin = {368, 100}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sensors.ForceSensor forceSensor1 annotation(
    Placement(transformation(origin = {368, 120}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Position position_2 annotation(
    Placement(transformation(origin = {356, 74}, extent = {{-6, -6}, {6, 6}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtm(rDisc = Wheel.diameter/2) annotation(
    Placement(transformation(origin = {152, 220}, extent = {{10, -20}, {-10, 20}})));
  GroundVehicleDynamics.Components.DiscEdgeBottomTranslation discEdgeBtm1(rDisc = Wheel1.diameter/2) annotation(
    Placement(transformation(origin = {302, 220}, extent = {{10, -20}, {-10, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {136, 152}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {290, 150}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel1(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {296, 98}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_0deg(animation = true, r = {cos(0*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(0*Modelica.Constants.pi/180)*Wheel.diameter/2}, color = {255, 0, 0}) annotation(
    Placement(transformation(origin = {128, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_40deg(animation = true, color = {255, 0, 0}, r = {cos(40*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(40*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {110, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_80deg(animation = true, color = {255, 0, 0}, r = {cos(80*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(80*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {94, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation strut_120deg(animation = true, color = {255, 0, 0}, r = {cos(120*Modelica.Constants.pi/180)*Wheel.diameter/2, 0, sin(120*Modelica.Constants.pi/180)*Wheel.diameter/2}) annotation(
    Placement(transformation(origin = {76, 242}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
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
  Modelica.Blocks.Tables.CombiTable2Ds Table_zGrd_Wheel(table = Glb_tblGrd) annotation(
    Placement(transformation(origin = {130, 98}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absolute(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {132, 220}, extent = {{5, -5}, {-5, 5}})));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheel(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {144, 192}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity w_absolute1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {321, 220}, extent = {{-5, -5}, {5, 5}}, rotation = -0)));
  GroundVehicleDynamics.Components.WheelGroundTangentialForce FtWheel1(rDisc = Glb_WheelDiameter/2) annotation(
    Placement(transformation(origin = {324, 190}, extent = {{-14, -4}, {14, 4}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor_ctr(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world, get_r = true, get_v = true, get_a = true, get_angles = true) annotation(
    Placement(transformation(origin = {217, 241}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles_ctr annotation(
    Placement(transformation(origin = {205, 241}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
  //-----------------------------------------------------------------
  /*
       * OpenModelica 用の地面可視化。
       *
       * Glb_tblGrd の各隣接4点から1セルを作り、
       * セル平均高さの薄い box を world 座標系に表示する。
       *
       * Glb_tblGrd[1, 2:end] : u2 = y 座標
       * Glb_tblGrd[2:end, 1] : u1 = x 座標
       * Glb_tblGrd[2:end, 2:end] : z=f(x,y)
       */
  parameter Integer nGrdCellX = size(Glb_tblGrd, 1) - 2 "x 方向セル数";
  parameter Integer nGrdCellY = size(Glb_tblGrd, 2) - 2 "y 方向セル数";
  parameter Real Glb_grdZMin = tableZMin(Glb_tblGrd) "Glb_tblGrd 内の z 最小値";
  parameter Real Glb_grdZMax = tableZMax(Glb_tblGrd) "Glb_tblGrd 内の z 最大値";
  parameter Real Glb_grdXMin = Glb_tblGrd[2, 1] "Glb_tblGrd 内の x 最小値";
  parameter Real Glb_grdXMax = Glb_tblGrd[size(Glb_tblGrd, 1), 1] "Glb_tblGrd 内の x 最大値";
  parameter Real Glb_grdYMin = Glb_tblGrd[1, 2] "Glb_tblGrd 内の Y 最小値";
  parameter Real Glb_grdYMax = Glb_tblGrd[1, size(Glb_tblGrd, 2)] "Glb_tblGrd 内の Y 最大値";
  parameter Real grdCellDX[nGrdCellX, nGrdCellY] = {{Glb_tblGrd[i + 2, 1] - Glb_tblGrd[i + 1, 1] for j in 1:nGrdCellY} for i in 1:nGrdCellX} "各セルの x 方向幅";
  parameter Real grdCellDY[nGrdCellX, nGrdCellY] = {{Glb_tblGrd[1, j + 2] - Glb_tblGrd[1, j + 1] for j in 1:nGrdCellY} for i in 1:nGrdCellX} "各セルの y 方向幅";
  parameter Real grdCellZ[nGrdCellX, nGrdCellY] = {{0.25*(Glb_tblGrd[i + 1, j + 1] + Glb_tblGrd[i + 2, j + 1] + Glb_tblGrd[i + 1, j + 2] + Glb_tblGrd[i + 2, j + 2]) for j in 1:nGrdCellY} for i in 1:nGrdCellX} "セル4頂点の z 平均値";
  parameter Modelica.Units.SI.Length Glb_grdVisThickness[nGrdCellX, nGrdCellY] = {{grdCellZ[i, j] - Glb_grdZMin for j in 1:nGrdCellY} for i in 1:nGrdCellX} "可視化 box の厚さ。物理モデルには影響しない";
  /*
      parameter Modelica.Units.SI.Length Glb_grdVisThickness[nGrdCellX, nGrdCellY] =
        {{0.02
          for j in 1:nGrdCellY} for i in 1:nGrdCellX}
        "可視化 box の厚さ。物理モデルには影響しない";
      */
  //-----
  parameter Real grdCellX0[nGrdCellX, nGrdCellY] = {{Glb_tblGrd[i + 1, 1] for j in 1:nGrdCellY} for i in 1:nGrdCellX} "各セルの x 下端";
  parameter Real grdCellY0[nGrdCellX, nGrdCellY] = {{Glb_tblGrd[1, j + 1] for j in 1:nGrdCellY} for i in 1:nGrdCellX} "各セルの y 下端";
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerXaxis(shapeType = "box", length = 1.1*(Glb_grdXMax), width = 0.04, height = 0.03, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 61}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerYaxis(shapeType = "box", length = 1.1*(Glb_grdYMax), width = 0.04, height = 0.03, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 39}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape markerZaxis(shapeType = "box", length = 10*(Glb_grdZMax), width = 0.04, height = 0.03, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, r_shape = {0, 0, 0}, color = {0, 0, 0}, specularCoefficient = 0.0) annotation(
    Placement(transformation(origin = {60, 18}, extent = {{-10, -10}, {10, 10}})));
  //-----------------------------------------------------------------
  TerrainCell terrainCell[nGrdCellX, nGrdCellY](x0 = grdCellX0, y0 = grdCellY0, zCenter = grdCellZ, dx = grdCellDX, dy = grdCellDY, each zMin = Glb_grdZMin, each zMax = Glb_grdZMax, thickness = Glb_grdVisThickness, each cellGap = 1e-5) annotation(
    Placement(transformation(origin = {46, 92}, extent = {{-10, -10}, {10, 10}})));
  /*
       * table のヘッダ行・ヘッダ列を除いた z データの最小値
       */

  function tableZMin
    input Real table[:, :];
    output Real zMin;
  algorithm
    zMin := table[2, 2];
    for i in 2:size(table, 1) loop
      for j in 2:size(table, 2) loop
        zMin := min(zMin, table[i, j]);
      end for;
    end for;
  end tableZMin;

  /*
       * table のヘッダ行・ヘッダ列を除いた z データの最大値
       */

  function tableZMax
    input Real table[:, :];
    output Real zMax;
  algorithm
    zMax := table[2, 2];
    for i in 2:size(table, 1) loop
      for j in 2:size(table, 2) loop
        zMax := max(zMax, table[i, j]);
      end for;
    end for;
  end tableZMax;

  /*
       * 連続 jet カラーマップ。
       *
       * zMin -> 濃青
       * zMax -> 濃赤
       *
       * zMin, zMax は常に入力 table の z データから取得するため、
       * テーブル値を変えても、その範囲全体が jet に写像される。
       */

  function jetColor
    input Real z;
    input Real zMin;
    input Real zMax;
    output Integer color[3];
  protected
    Real s;
    Real red;
    Real green;
    Real blue;
  algorithm
    if zMax > zMin then
      s := (z - zMin)/(zMax - zMin);
    else
      s := 0.5;
    end if;
    s := min(1.0, max(0.0, s));
    red := min(1.0, max(0.0, 1.5 - abs(4.0*s - 3.0)));
    green := min(1.0, max(0.0, 1.5 - abs(4.0*s - 2.0)));
    blue := min(1.0, max(0.0, 1.5 - abs(4.0*s - 1.0)));
    color[1] := integer(255*red);
    color[2] := integer(255*green);
    color[3] := integer(255*blue);
  end jetColor;

  //-----------------------------------------------------------------
  /*
       * 地面タイル1枚。
       *
       * frame_a を world.frame_b へ接続すると、
       * xCenter, yCenter, zCenter が world 座標として使われる。
       */

  model TerrainCell
    parameter Modelica.Units.SI.Length x0 "セルの x 下端";
    parameter Modelica.Units.SI.Length y0 "セルの y 下端";
    parameter Modelica.Units.SI.Length zCenter;
    parameter Modelica.Units.SI.Length dx(min = 0);
    parameter Modelica.Units.SI.Length dy(min = 0);
    //parameter Modelica.Units.SI.Length thickness(min = 0.001) = 0.02;
    parameter Modelica.Units.SI.Length thickness(min = 0.001);
    parameter Modelica.Units.SI.Length cellGap = 1e-5 "0.01 mm。最小セル幅 1 mm より十分小さい";
    parameter Real zMin;
    parameter Real zMax;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-94, 2}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape box(shapeType = "box", length = dxVisual, width = dyVisual, height = thickness, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0},  /*
       * r_shape は OpenModelica では box の始点として扱う。
       * したがって、テーブル格子端点 + gap/2 を直接使う。
       */
       r_shape = {x0 + cellGap/2, y0 + dy/2, zCenter - thickness/2}, color = jetColor(zCenter, zMin, zMax), specularCoefficient = 0.0);
    
  protected
    parameter Modelica.Units.SI.Length dxVisual = max(1e-8, dx - cellGap);
    parameter Modelica.Units.SI.Length dyVisual = max(1e-8, dy - cellGap);
  equation
    connect(frame_a, box.frame_a);
  annotation(
      Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}})}));
end TerrainCell;

  //-----------------------------------------------------------------
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
    Line(points = {{198, 116}, {198, 121}}, color = {0, 127, 0}));
  connect(position.flange, forceSensor.flange_b) annotation(
    Line(points = {{188, 134}, {198, 134}, {198, 131}}, color = {0, 127, 0}));
  connect(const[1].y, force.force[1]) annotation(
    Line(points = {{200, 145}, {208.4, 145}, {208.4, 174}, {208, 174}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const[2].y, force.force[2]) annotation(
    Line(points = {{200, 145}, {208.4, 145}, {208.4, 174}, {208, 174}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceSensor.f, force.force[3]) annotation(
    Line(points = {{203.5, 122}, {208, 122}, {208, 174}}, color = {0, 0, 127}));
  connect(const2[1].y, force1.force[1]) annotation(
    Line(points = {{375, 140}, {384.4, 140}, {384.4, 174}, {384, 174}}, color = {0, 0, 127}, thickness = 0.5));
  connect(const2[2].y, force1.force[2]) annotation(
    Line(points = {{375, 140}, {384.4, 140}, {384.4, 174}, {384, 174}}, color = {0, 0, 127}, thickness = 0.5));
  connect(elastoGap1.flange_b, forceSensor1.flange_a) annotation(
    Line(points = {{368, 110}, {368, 115}}, color = {0, 127, 0}));
  connect(position1.flange, forceSensor1.flange_b) annotation(
    Line(points = {{358, 129}, {368, 129}, {368, 125}}, color = {0, 127, 0}));
  connect(forceSensor1.f, force1.force[3]) annotation(
    Line(points = {{373.5, 116}, {373.5, 115.5}, {383.5, 115.5}, {383.5, 174}, {384, 174}}, color = {0, 0, 127}));
  connect(position_2.flange, elastoGap1.flange_a) annotation(
    Line(points = {{362, 74}, {368, 74}, {368, 90}}, color = {0, 127, 0}));
  connect(Wheel.frame_b, discEdgeBtm.frame_a) annotation(
    Line(points = {{166, 220}, {152, 220}}, color = {95, 95, 95}));
  connect(force.frame_b, discEdgeBtm.frame_btm) annotation(
    Line(points = {{208, 196}, {208, 200}, {152, 200}}, color = {95, 95, 95}));
  connect(Wheel1.frame_a, discEdgeBtm1.frame_a) annotation(
    Line(points = {{284, 220}, {302, 220}}, color = {95, 95, 95}));
  connect(force1.frame_b, discEdgeBtm1.frame_btm) annotation(
    Line(points = {{384, 196}, {384, 200}, {302, 200}}, color = {95, 95, 95}));
  connect(discEdgeBtm.frame_btm, absolutePosition.frame_a) annotation(
    Line(points = {{152, 200}, {136, 200}, {136, 158}}, color = {95, 95, 95}));
  connect(discEdgeBtm1.frame_btm, absolutePosition1.frame_a) annotation(
    Line(points = {{302, 200}, {290, 200}, {290, 156}}, color = {95, 95, 95}));
  connect(absolutePosition1.r[1], Table_zGrd_Wheel1.u1) annotation(
    Line(points = {{290, 143}, {290, 126.2}, {302, 126.2}, {302, 109.4}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[2], Table_zGrd_Wheel1.u2) annotation(
    Line(points = {{290, 143}, {290, 109.4}}, color = {0, 0, 127}));
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
    Line(points = {{136, 145}, {136, 134}, {175, 134}}, color = {0, 0, 127}));
  connect(absolutePosition1.r[3], position1.s_ref) annotation(
    Line(points = {{290, 143}, {290, 129}, {345, 129}}, color = {0, 0, 127}));
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
    Line(points = {{136, 145}, {136, 110}}, color = {0, 0, 127}));
  connect(absolutePosition.r[2], Table_zGrd_Wheel.u2) annotation(
    Line(points = {{136, 145}, {136, 142}, {124, 142}, {124, 110}}, color = {0, 0, 127}));
  connect(Table_zGrd_Wheel.y, position2.s_ref) annotation(
    Line(points = {{130, 88}, {130, 74}, {179, 74}}, color = {0, 0, 127}));
  connect(discEdgeBtm.frame_a, w_absolute.frame_a) annotation(
    Line(points = {{152, 220}, {137, 220}}, color = {95, 95, 95}));
  connect(FtWheel.frame_b, discEdgeBtm.frame_btm) annotation(
    Line(points = {{144, 190}, {144, 200}, {152, 200}}, color = {95, 95, 95}));
  connect(forceSensor.f, FtWheel.u_Fn) annotation(
    Line(points = {{203.5, 122}, {208, 122}, {208, 166}, {144, 166}, {144, 187}}, color = {0, 0, 127}));
  connect(discEdgeBtm1.frame_a, w_absolute1.frame_a) annotation(
    Line(points = {{302, 220}, {316, 220}}, color = {95, 95, 95}));
  connect(discEdgeBtm1.frame_btm, FtWheel1.frame_b) annotation(
    Line(points = {{302, 200}, {324, 200}, {324, 188}}, color = {95, 95, 95}));
  connect(forceSensor1.f, FtWheel1.u_Fn) annotation(
    Line(points = {{373.5, 116}, {384, 116}, {384, 166}, {324, 166}, {324, 186}}, color = {0, 0, 127}));
  connect(bodyCenter.frame_a, absoluteSensor_ctr.frame_a) annotation(
    Line(points = {{230, 230}, {217, 230}, {217, 236}}, color = {95, 95, 95}));
  connect(bodyCenter.frame_a, absoluteAngles_ctr.frame_a) annotation(
    Line(points = {{230, 230}, {205, 230}, {205, 236}}, color = {95, 95, 95}));
  connect(w_absolute.w[2], FtWheel.u_wRoll) annotation(
    Line(points = {{126, 220}, {126, 208}, {130, 208}, {130, 196}}, color = {0, 0, 127}));
  connect(w_absolute1.w[2], FtWheel1.u_wRoll) annotation(
    Line(points = {{326, 220}, {326, 206}, {310, 206}, {310, 194}}, color = {0, 0, 127}));
//-----------------------------------------------------------------
  /*for i in 1:nGrdCellX loop
    for j in 1:nGrdCellY loop
      connect(world.frame_b, terrainCell[i, j].frame_a);
    end for;
  end for;*/
//-----------------------------------------------------------------
  connect(world.frame_b, markerXaxis.frame_a) annotation(
    Line(points = {{26, 21}, {38, 21}, {38, 61}, {50, 61}}, color = {95, 95, 95}));
  connect(markerYaxis.frame_a, world.frame_b) annotation(
    Line(points = {{50, 40}, {38, 40}, {38, 21}, {26, 21}}, color = {95, 95, 95}));
  connect(world.frame_b, markerZaxis.frame_a) annotation(
    Line(points = {{26, 21}, {38, 21}, {38, 18}, {50, 18}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-40, 260}, {500, 0}})),
    version = "",
    uses(Modelica(version = "4.1.0")),
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-09, Interval = 0.02),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end ProtoPanjan_004;