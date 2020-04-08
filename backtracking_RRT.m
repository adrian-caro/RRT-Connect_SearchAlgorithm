function [camino]=backtracking_RRT(arbol,xo,yo)

% Recibe los nodos que forman parte del camino desde el nodo goal al nodo
% origen y los coloca al revés (de nodo origen a nodo goal).

v=[];               % Vector auxiliar
camino=[1;1];       % Hilo de salida (ordenado)
i=length(arbol.x);  % Indice del nodo solución (goal, ultimo del vector)
contador=1;

while arbol.p(i)~=0
    
    % Dado el indice del padre, encontramos el siguiente nodo
    
    v(1,contador)=arbol.x(i);
    v(2,contador)=arbol.y(i);
    
    i=arbol.p(i);
    contador=contador+1;
    
end

% Ultimo nodo del auxiliar es el nodo origen.
v(1,contador)=xo;
v(2,contador)=yo;

for i=1:size(v,2) % Da la vuelta a v para tener el camino ordenado.
    camino(1,i)=v(1,size(v,2)+1-i);
    camino(2,i)=v(2,size(v,2)+1-i);
end

end
