within GroundVehicleDynamics.Visualization;

model VisTerrainMultiStlContour "Display pre-split terrain STL meshes with n automatically assigned contour colors"
  import MB = Modelica.Mechanics.MultiBody;
  parameter Integer n(min = 1) = 10 "Number of height bands / STL files";
  parameter String filePrefix = "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1" "File prefix including directory; file i is filePrefix + String(i) + fileSuffix";
  parameter String fileSuffix = ".stl" "File suffix (default .stl)";
  parameter Integer firstIndex = 1 "Index of first file: use 0 if generated filenames start at 0";
  parameter Boolean enableAnimation = true;
  parameter Real specularCoefficient = 0.0;
  parameter Real zMin = 0 "Height represented by the first (lowest) color band";
  parameter Real zMax = 1 "Height represented by the last (highest) color band";
  MB.Interfaces.Frame_a frame_a "All meshes share this reference frame" annotation(
    Placement(transformation(extent = {{-116, -16}, {-84, 16}}), iconTransformation(extent = {{-116, -16}, {-84, 16}})));
  final parameter MB.Types.Color bandColor[n] = {jetColor(if n == 1 then 0.5 else (i - 1.0)/(n - 1.0)) for i in 1:n} "RGB colors, low bands blue and high bands red";
  // Every STL must be generated in the same world coordinates, and its
  // filename must match filePrefix + String(firstIndex+i-1) + fileSuffix.
  MB.Visualizers.FixedShape terrainBand[n](shapeType = {filePrefix + String(firstIndex + i - 1) + fileSuffix for i in 1:n}, each r_shape = {0, 0, 0}, each lengthDirection = {1, 0, 0}, each widthDirection = {0, 1, 0}, each extra = 0, color = bandColor, each specularCoefficient = specularCoefficient, each animation = enableAnimation) "One external mesh per height band";

  function jetColor "RGB jet ramp: navy -> cyan -> green -> yellow -> dark red"
    input Real fraction "Normalized band position in [0,1]";
    output MB.Types.Color rgb;
  protected
    Real s;
  algorithm
    s := min(1.0, max(0.0, fraction));
    rgb := {integer(255*min(1.0, max(0.0, 1.5 - abs(4*s - 3)))), integer(255*min(1.0, max(0.0, 1.5 - abs(4*s - 2)))), integer(255*min(1.0, max(0.0, 1.5 - abs(4*s - 1))))};
  end jetColor;
equation
  for i in 1:n loop
    connect(frame_a, terrainBand[i].frame_a);
  end for;
  annotation(
    defaultComponentName = "visTerrain",
    Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(fillColor = {235, 245, 240}, fillPattern = FillPattern.Solid, extent = {{-100, 80}, {100, -80}}), Polygon(fillColor = {25, 180, 120}, fillPattern = FillPattern.Solid, points = {{-90, -55}, {-50, -10}, {-10, -20}, {30, 45}, {85, 0}, {85, -55}, {-90, -55}}), Text(origin = {0, 43}, extent = {{-99, 56}, {99, 41}}, textString = "%name"), Text(origin = {0, -29}, extent = {{-99, -71}, {99, -56}}, textString = "n=%n")}),
    Documentation(info = "<html><p>Loads n pre-generated STL terrain meshes as FixedShape objects. All meshes are fixed to the same frame_a, use original mesh coordinates (extra=0), and receive a jet color according to ascending file index. File i uses filePrefix + String(firstIndex+i-1) + fileSuffix. zMin/zMax document the corresponding physical height span; the color ramp is assigned by band index, not by reading STL geometry. For portable modelica:// URIs, place the generated files in the loaded GroundVehicleDynamics library before translation/animation. STL color override is renderer-dependent: verify that your OMEdit build applies the shape color to imported STL files.</p></html>"));
end VisTerrainMultiStlContour;