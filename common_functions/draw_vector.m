function draw_vector(startP, endP, style)

quiver3(startP(1),startP(2),startP(3),endP(1)-startP(1),endP(2)-startP(2),endP(3)-startP(3),style,'ShowArrowHead','off','AutoScale','off','LineWidth',1.5);

end

