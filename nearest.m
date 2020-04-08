function [x,y,ganador]=nearest(rx,ry,arbol)

% Recibe como parámetros un punto (x,y) y una estructura árbol.
% Va uno a uno recorriendo los puntos del árbol y calculando su
% distancia respecto al punto proporcionado. Devuelve el punto 
% del árbol más cercano a dicho punto y su índice (posición) 
% dentro de la estructura árbol.

ganador=0;
distanciaMinima=inf;

for i=1:length(arbol.x)
    dist=sqrt((arbol.x(i)-rx)^2+(arbol.y(i)-ry)^2);
    if dist<distanciaMinima
        distanciaMinima=dist;
        ganador=i;
    end
end

x=arbol.x(ganador);
y=arbol.y(ganador);

end