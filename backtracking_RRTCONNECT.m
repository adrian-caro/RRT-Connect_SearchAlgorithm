function [v]=backtracking_RRTCONNECT(arbol,xo,yo)

% Recibe una estructura árbol y su punto de origen. Devuelve el vector 
% de puntos solución, ordenados desde el origen al nodo final.
% Empieza en el punto final del árbol, guardando el punto en un vector 
% auxiliar, y dirigiéndose a su punto padre (posición referenciada en 
% su componente "padre" de la estructura). Lo repite sucesivamente hasta
% llegar a un punto cuyo padre sea el nodo inicial. Finalmente se añade 
% el nodo inicial al vector auxiliar. 
% Dado que el vector auxiliar va de final a inicio, se procede a darle la
% vuelta para obtener un hilo desde el punto de origen hasta el punto final.
% En el caso del ‘backtracking’ para el RRT-Connect, no se realiza el paso
% de darle la vuelta al vector, ya que esto va a depender de si se trata del
% árbol origen o el árbol destino. La inversión de este hilo se realiza de
% forma externa en la propia función del RRT-Connect.


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
% 
% for i=1:size(v,2) % Da la vuelta a v para tener el camino ordenado.
%     camino(1,i)=v(1,size(v,2)+1-i);
%     camino(2,i)=v(2,size(v,2)+1-i);
% end

end
