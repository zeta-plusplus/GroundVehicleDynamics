within GroundVehicleDynamics.Visualization;

model TerrainTableVisualizer "CombiTable2D 形式の地形テーブルを FixedShape box 群として可視化する"
  import SI = Modelica.Units.SI;

  parameter Boolean tableOnFile = false
    "true: fileName/tableName の外部テーブルを使用。false: table parameter を使用";
  parameter Real table[:, :] = [0, 0, 1; 0, 0, 0; 1, 0, 0]
    "tableOnFile=false 時に用いる CombiTable2D/CombiTable2Ds 形式の地形データ";
  parameter String tableName = "NoName"
    "tableOnFile=true 時に読むテーブル名。テキストファイルの 'double <name>(n,m)' と一致させる";
  parameter String fileName = "NoName"
    "tableOnFile=true 時に読むテキスト/MAT ファイル。modelica:// URI は loadResource で解決して渡す";
  parameter String delimiter = ","
    "テキストテーブルの区切り文字。タブ区切りなら \"\\t\"、空白区切りなら \" \"";
  parameter Modelica.Blocks.Types.Smoothness smoothness =
      Modelica.Blocks.Types.Smoothness.LinearSegments
    "内部 CombiTable2D の補間方式";
  parameter Modelica.Blocks.Types.Extrapolation extrapolation =
      Modelica.Blocks.Types.Extrapolation.LastTwoPoints
    "内部 CombiTable2D の外挿方式";
  parameter Boolean verboseRead = false
    "true の場合、CombiTable2D の読込み情報を表示";

  parameter SI.Length visualThickness[nCellU1, nCellU2] =
      {{zCell[i, j] - zMin for j in 1:nCellU2} for i in 1:nCellU1}
    "可視化 box の厚さ。物理モデルには影響しない";
  parameter SI.Length cellGap = 0
    "隣接セル間の描画用隙間。物理モデルには影響しない";
  parameter Real zScale = 1.0
    "描画上の z 倍率。色計算には影響しない";
  parameter Boolean enableAnimation = true
    "false の場合、全地形セルを animation 非表示にする";
  parameter Real specularCoefficient = 0.0
    "鏡面反射係数。コンター色を見やすくするなら 0 推奨";

  /*
   * lookup の入れ子インスタンス。
   * これにより tableOnFile/tableName/fileName/smoothness/extrapolation は
   * Modelica.Blocks.Tables.CombiTable2D と同じ意味で機能する。
   *
   * tableData は visualization 用の格子値を parameter matrix として保持する。
   * 外部ファイル使用時にも readRealMatrix で同じ #1/double tableName(n,m)
   * 書式を読む。寸法は tableSize を先に指定する必要がある。
   */
  parameter Integer tableSize[2] = if tableOnFile then {3, 3} else size(table)
    "外部ファイルの table 行数・列数。CombiTable2D table 全体（軸行・軸列を含む）の寸法";
  parameter Real tableData[tableSize[1], tableSize[2]] =
    if tableOnFile then
      Modelica.Utilities.Streams.readRealMatrix(fileName, tableName,
        tableSize[1], tableSize[2])
    else table
    "可視化用に保持する CombiTable2D 形式の完全テーブル";
  
  parameter Real dmyu1=1;
  parameter Real dmyu2=1;
  Modelica.Blocks.Tables.CombiTable2Ds terrainLookup(
    tableOnFile = tableOnFile,
    table = table,
    tableName = tableName,
    fileName = fileName,
    delimiter = delimiter,
    smoothness = smoothness,
    extrapolation = extrapolation,
    verboseRead = verboseRead)
    "地形高さ lookup。本コンポーネントの内部実装として保持";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "地形を配置する基準座標系"
    annotation(Placement(transformation(origin = {-94, 2}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

  final parameter Integer nU1 = size(tableData, 1) - 1
    "u1 方向の地形格子点数";
  final parameter Integer nU2 = size(tableData, 2) - 1
    "u2 方向の地形格子点数";
  final parameter Integer nCellU1 = nU1 - 1
    "u1 方向の地形セル数";
  final parameter Integer nCellU2 = nU2 - 1
    "u2 方向の地形セル数";
  final parameter Real zMin = tableZMin(tableData)
    "テーブルの z 最小値";
  final parameter Real zMax = tableZMax(tableData)
    "テーブルの z 最大値";

  final parameter SI.Position u1Start[nCellU1, nCellU2] =
    {{tableData[i + 1, 1] for j in 1:nCellU2} for i in 1:nCellU1};
  final parameter SI.Position u2Start[nCellU1, nCellU2] =
    {{tableData[1, j + 1] for j in 1:nCellU2} for i in 1:nCellU1};
  final parameter SI.Length dU1[nCellU1, nCellU2] =
    {{tableData[i + 2, 1] - tableData[i + 1, 1] for j in 1:nCellU2} for i in 1:nCellU1};
  final parameter SI.Length dU2[nCellU1, nCellU2] =
    {{tableData[1, j + 2] - tableData[1, j + 1] for j in 1:nCellU2} for i in 1:nCellU1};
  final parameter SI.Position zCell[nCellU1, nCellU2] =
    {{zScale*0.25*(tableData[i + 1, j + 1] + tableData[i + 2, j + 1] +
                    tableData[i + 1, j + 2] + tableData[i + 2, j + 2])
      for j in 1:nCellU2} for i in 1:nCellU1};

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

  model TerrainCell "TerrainTableVisualizer 内部用の地形タイル1枚"
    parameter SI.Position u1Start "セルの u1/x 方向の始点";
    parameter SI.Position u2Start "セルの u2/y 方向の始点";
    parameter SI.Position zCenter "セル四隅の平均から求めた描画高さ";
    parameter SI.Length dU1(min = 0) "元テーブルにおける u1/x 方向セル幅";
    parameter SI.Length dU2(min = 0) "元テーブルにおける u2/y 方向セル幅";
    parameter SI.Length visualThickness(min = 1e-8) = 0.02 "box の描画用厚さ";
    parameter SI.Length cellGap(min = 0) = 1e-5 "隣接セル間の描画用 gap";
    parameter Real zMin;
    parameter Real zMax;
    parameter Real zScale = 1;
    parameter Boolean enableAnimation = true;
    parameter Real specularCoefficient = 0.0;
    final parameter SI.Length dU1Visual = max(Modelica.Constants.small, dU1 - cellGap);
    final parameter SI.Length dU2Visual = max(Modelica.Constants.small, dU2 - cellGap);

    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-94, 2}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape(
      shapeType = "box",
      length = dU1Visual,
      width = dU2Visual,
      height = visualThickness,
      lengthDirection = {1, 0, 0},
      widthDirection = {0, 1, 0},
      r_shape = {u1Start + cellGap/2, u2Start + dU2/2, zCenter - visualThickness/2},
      color = jetColor(zCenter/zScale, zMin, zMax),
      specularCoefficient = specularCoefficient,
      animation = enableAnimation);
  equation
    connect(frame_a, shape.frame_a);
  end TerrainCell;

  TerrainCell cell[nCellU1, nCellU2](
    u1Start = u1Start,
    u2Start = u2Start,
    zCenter = zCell,
    dU1 = dU1,
    dU2 = dU2,
    each zMin = zMin,
    each zMax = zMax,
    visualThickness = visualThickness,
    each cellGap = cellGap,
    each enableAnimation = enableAnimation,
    each specularCoefficient = specularCoefficient,
    each zScale = zScale);

protected
  final parameter SI.Length minCellSize = min(min(dU1), min(dU2));

initial algorithm
  assert(size(tableData, 1) >= 3,
    "TerrainTableVisualizer: table must have at least 3 rows. A header row and at least two u1 grid points are required.");
  assert(size(tableData, 2) >= 3,
    "TerrainTableVisualizer: table must have at least 3 columns. A header column and at least two u2 grid points are required.");
  assert(min(dU1) > 0,
    "TerrainTableVisualizer: u1 coordinates in table[:,1] must be strictly increasing.");
  assert(min(dU2) > 0,
    "TerrainTableVisualizer: u2 coordinates in table[1,:] must be strictly increasing.");
  assert(cellGap >= 0,
    "TerrainTableVisualizer: cellGap must be non-negative.");
  assert(cellGap < minCellSize,
    "TerrainTableVisualizer: cellGap must be smaller than the smallest cell width. cellGap = " +
    String(cellGap) + " m, minimum cell width = " + String(minCellSize) + " m.");

equation
  for i in 1:nCellU1 loop
    for j in 1:nCellU2 loop
      connect(frame_a, cell[i, j].frame_a);
    end for;
  end for;

  terrainLookup.u1=dmyu1;
  terrainLookup.u2=dmyu2;
  
  annotation(
    defaultComponentName = "VisTerrainTbl",
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, -113}, extent = {{-100, 9}, {100, -9}}, textString = "%name")}, coordinateSystem(preserveAspectRatio = false)));
end TerrainTableVisualizer;