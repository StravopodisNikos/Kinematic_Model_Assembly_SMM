function PointsXYZ=fetchpoints(seq)

points=[0.25 0.1915 0.0434 -0.125 -0.2349 -0.2349 -0.125 0.0434 0.1915 0.25; 0 0.0964 0.1477 0.1299 0.0513 -0.0513 -0.1299 -0.1477 -0.0964 0; 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15];

task_orientation=eye(3);
for i=1:length(seq)
    Points{i}(1:3,1:3)=task_orientation;
    Points{i}(:,4)=points(:,seq(i));
    Points{i}(4,:)=[0 0 0 1];
    
    PointsXYZ{i} = Points{i}(1:3,4);
end

figure;
point_number = string(1:10);
for i=1:length(seq)
    scatter3(PointsXYZ{i}(1), PointsXYZ{i}(2), PointsXYZ{i}(3),80);  hold on;
    textscatter3( [PointsXYZ{i}(1) PointsXYZ{i}(2) PointsXYZ{i}(3)], point_number(i) ); hold on;
end
    