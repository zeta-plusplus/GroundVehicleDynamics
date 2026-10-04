within GroundVehicleDynamics.Examples.ComponentUsage;

model VisTerrainMultiStlContour_ex01
  extends Modelica.Icons.Example;
  
  parameter TerrainSTL terrainSTL;
  Visualization.VisTerrainMultiStlContour visTerrain(stlFileName = terrainSTL.fileName, n = terrainSTL.n)  annotation(
    Placement(transformation(origin = {66, 24}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(animateGround = true, groundColor = {130, 200, 130}, groundLength_u = 4, label2 = "z", n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {74, 14}, extent = {{-60, 0}, {-40, 20}})));

  record TerrainSTL
    parameter Integer n = 10;
    
    parameter String fileName[n]={
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[1].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[2].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[3].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[4].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[5].stl",
      
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[6].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[7].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[8].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[9].stl",
      "modelica://GroundVehicleDynamics/Examples/Projects/Panjan/TerrainData/tableGrd_z_xy_wave_1[10].stl"
    };  
    
  end TerrainSTL;
equation
  connect(world.frame_b, visTerrain.frame_a) annotation(
    Line(points = {{34, 24}, {56, 24}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.1),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
  Diagram(coordinateSystem(extent = {{0, 40}, {80, 0}})));
end VisTerrainMultiStlContour_ex01;