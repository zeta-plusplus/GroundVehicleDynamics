within GroundVehicleDynamics.Visualization;

model TerrainTableVisualizer "CombiTable2D 形式の地形テーブルを FixedShape box 群として可視化する"
  import SI = Modelica.Units.SI;
  /*
     * CombiTable2D / CombiTable2Ds の table 形式:
     *
     *              u2[1]   u2[2]   ...  u2[nv]
     *   u1[1]      z11     z12     ...  z1,nv
     *   u1[2]      z21     z22     ...  z2,nv
     *     :
     *   u1[nu]     znu,1   znu,2   ...  znu,nv
     *
     * table[1, 2:end] : u2 座標
     * table[2:end, 1] : u1 座標
     * table[2:end, 2:end] : z = f(u1,u2)
     */
  parameter Real table[:, :] = [0, 0, 1; 0, 0, 0; 1, 0, 0] "CombiTable2D/CombiTable2Ds 形式の地形データ";
  parameter SI.Length visualThickness = 0.02 "各地形セル box の描画用厚さ";
  parameter SI.Length cellGap = 0 "隣接セル間の描画用隙間。物理モデルには影響しない";
  parameter Real zScale = 1.0 "描画上の z 倍率。色計算には影響しない";
  parameter Boolean enableAnimation = true "false の場合、全地形セルを animation 非表示にする";
  parameter Real specularCoefficient = 0.0 "鏡面反射係数。コンター色を見やすくするなら 0 推奨";
  /*
     * table の先頭行・先頭列は座標軸なので、
     * 実際の地形格子点数はそれぞれ 1 を引いた値になる。
     */
  final parameter Integer nU1 = size(table, 1) - 1 "u1 方向の地形格子点数";
  final parameter Integer nU2 = size(table, 2) - 1 "u2 方向の地形格子点数";
  final parameter Integer nCellU1 = nU1 - 1 "u1 方向の地形セル数";
  final parameter Integer nCellU2 = nU2 - 1 "u2 方向の地形セル数";
  final parameter Real zMin = tableZMin(table) "テーブルの z 最小値";
  final parameter Real zMax = tableZMax(table) "テーブルの z 最大値";
  /*
     * 各セルの u1 方向始点。
     *
     * table の第1列:
     * table[2,1], table[3,1], ... が u1 座標。
     */
  final parameter SI.Position u1Start[nCellU1, nCellU2] = {{table[i + 1, 1] for j in 1:nCellU2} for i in 1:nCellU1};
  /*
     * 各セルの u2 方向始点。
     *
     * table の第1行:
     * table[1,2], table[1,3], ... が u2 座標。
     */
  final parameter SI.Position u2Start[nCellU1, nCellU2] = {{table[1, j + 1] for j in 1:nCellU2} for i in 1:nCellU1};
  /*
     * 各セルの x/u1 方向幅。
     */
  final parameter SI.Length dU1[nCellU1, nCellU2] = {{table[i + 2, 1] - table[i + 1, 1] for j in 1:nCellU2} for i in 1:nCellU1};
  /*
     * 各セルの y/u2 方向幅。
     */
  final parameter SI.Length dU2[nCellU1, nCellU2] = {{table[1, j + 2] - table[1, j + 1] for j in 1:nCellU2} for i in 1:nCellU1};
  /*
     * 各セルの表示高さ。
     *
     * box は連続曲面ではないため、セル四隅の平均 z を
     * タイル高さとして使用する。
     */
  final parameter SI.Position zCell[nCellU1, nCellU2] = {{zScale*0.25*(table[i + 1, j + 1] + table[i + 2, j + 1] + table[i + 1, j + 2] + table[i + 2, j + 2]) for j in 1:nCellU2} for i in 1:nCellU1};
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "地形を配置する基準座標系" annotation(
    Placement(transformation(origin = {-94, 2}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  /*
     * 以下は TerrainTableVisualizer 内部専用。
     */

  function tableZMin "CombiTable2D table の z 部分だけを対象とする最小値関数"
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

  function tableZMax "CombiTable2D table の z 部分だけを対象とする最大値関数"
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

  function jetColor "zMin を濃青、zMax を濃赤とする連続 jet カラーマップ"
    input Real z;
    input Real zMin;
    input Real zMax;
    output Integer color[3] "RGB color, each element is in 0...255";
  protected
    Real s;
    Real red;
    Real green;
    Real blue;
  algorithm
/*
     * z の実際の値は符号・単位・オフセットを問わない。
     * 常に table 内の z 最小値・最大値を jet の端点とする。
     */
    if zMax > zMin then
      s := (z - zMin)/(zMax - zMin);
    else
/*
       * 一様平面の場合。jet の中央色を使う。
       */
      s := 0.5;
    end if;
    s := min(1.0, max(0.0, s));
/*
     * MATLAB 系 jet に相当する連続擬似カラー。
     *
     * s=0.00 : dark blue
     * s=0.25 : blue/cyan
     * s=0.50 : green
     * s=0.75 : yellow
     * s=1.00 : dark red
     */
    red := min(1.0, max(0.0, 1.5 - abs(4.0*s - 3.0)));
    green := min(1.0, max(0.0, 1.5 - abs(4.0*s - 2.0)));
    blue := min(1.0, max(0.0, 1.5 - abs(4.0*s - 1.0)));
    color[1] := integer(255*red);
    color[2] := integer(255*green);
    color[3] := integer(255*blue);
  end jetColor;

  model TerrainCell "TerrainTableVisualizer 内部用の地形タイル1枚"
    parameter SI.Position u1Start "セルの u1/x 方向の始点";
    parameter SI.Position u2Start "セルの u2/y 方向の始点";
    parameter SI.Position zCenter "セル四隅の平均から求めた描画高さ";
    parameter SI.Length dU1(min = 0) "元テーブルにおける u1/x 方向セル幅";
    parameter SI.Length dU2(min = 0) "元テーブルにおける u2/y 方向セル幅";
    parameter SI.Length visualThickness(min = 1e-8) = 0.02 "box の描画用厚さ";
    parameter SI.Length cellGap(min = 0) = 0 "隣接セル間の描画用 gap";
    parameter Real zMin;
    parameter Real zMax;
    parameter Real zScale=1;
    parameter Boolean enableAnimation = true;
    parameter Real specularCoefficient = 0.0;
    /*
         * 表示用の長さ・幅。
         *
         * cellGap は両側に半分ずつ配分するため、
         * 実際の表示寸法は dU1-cellGap, dU2-cellGap となる。
         */
    final parameter SI.Length dU1Visual = max(Modelica.Constants.small, dU1 - cellGap);
    final parameter SI.Length dU2Visual = max(Modelica.Constants.small, dU2 - cellGap);
    /**/
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-94, 2}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape(shapeType = "box", length = dU1Visual, width = dU2Visual, height = visualThickness, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0},  /*
       * OpenModelica の box 表示に合わせた位置指定:
       *
       * u1/x:
       *   shape の length は r_shape[1] を始点として +x に伸びる。
       *
       * u2/y:
       *   shape の width は r_shape[2] を中心として ±y に広がる。
       *
       * よって u2/y については、元セル中心 u2Start+dU2/2 を
       * r_shape[2] に指定する。
       */
       r_shape = {u1Start + cellGap/2, u2Start + dU2/2, zCenter}, color = jetColor(zCenter/zScale, zMin, zMax), specularCoefficient = specularCoefficient, animation = enableAnimation);
  equation
    connect(frame_a, shape.frame_a);
  end TerrainCell;

  /*
     * 地形セル配列。
     *
     * 上位モデルが直接触る必要はない。
     * table の格子数に応じて自動生成される。
     */
  TerrainCell cell[nCellU1, nCellU2](u1Start = u1Start, u2Start = u2Start, zCenter = zCell, dU1 = dU1, dU2 = dU2, each zMin = zMin, each zMax = zMax, each visualThickness = visualThickness, each cellGap = cellGap, each enableAnimation = enableAnimation, each specularCoefficient = specularCoefficient, each zScale= zScale);
protected
  /*
     * cellGap が局所セル寸法以上なら、表示寸法が負になる。
     * それを早期に検出するための値。
     */
  final parameter SI.Length minCellSize = min(min(dU1), min(dU2));
initial algorithm
  assert(size(table, 1) >= 3, "TerrainTableVisualizer: table must have at least 3 rows. " + "A header row and at least two u1 grid points are required.");
  assert(size(table, 2) >= 3, "TerrainTableVisualizer: table must have at least 3 columns. " + "A header column and at least two u2 grid points are required.");
  assert(min(dU1) > 0, "TerrainTableVisualizer: u1 coordinates in table[:,1] " + "must be strictly increasing.");
  assert(min(dU2) > 0, "TerrainTableVisualizer: u2 coordinates in table[1,:] " + "must be strictly increasing.");
  assert(cellGap >= 0, "TerrainTableVisualizer: cellGap must be non-negative.");
  assert(cellGap < minCellSize, "TerrainTableVisualizer: cellGap must be smaller than the " + "smallest cell width. cellGap = " + String(cellGap) + " m, minimum cell width = " + String(minCellSize) + " m.");
equation
/*
   * 全セルを同一の基準フレームに固定する。
   *
   * frame_a を world.frame_b へ接続すれば、全セルは
   * world 座標系で table の座標に従って配置される。
   */
  for i in 1:nCellU1 loop
    for j in 1:nCellU2 loop
      connect(frame_a, cell[i, j].frame_a);
    end for;
  end for;
  
  
  annotation(
    defaultComponentName = "VisTerrainTbl",
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, -113}, extent = {{-100, 9}, {100, -9}}, textString = "%name")}, coordinateSystem(preserveAspectRatio = false)));
  
  
end TerrainTableVisualizer;
