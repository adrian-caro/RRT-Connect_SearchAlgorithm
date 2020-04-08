function [hilo]=RRT()


% Genera puntos aleatorios e intenta conectar con el punto más cercano de 
% un árbol que tiene como inicio el punto origen. Discretiza la recta de 
% unión añadiendo los puntos que no chocan. En el caso de que algún punto 
% choque, añade el segmento que lo precede al árbol y vuelve a crear otro 
% punto aleatorio. El algoritmo se repite un número configurado de veces.
% Se establece que cada cierto número de iteraciones, en vez de con un punto
% aleatorio, el árbol se intente conectar con el punto de destino. En el 
% caso de conseguirlo, realiza el ‘backtracking’ (función ‘backtracking.m’)
% y devuelve como parámetro una vector de puntos solución desde el origen 
% al punto final.




global A B xg yg xo yo t

hold on;
daspect([1 1 1])
axis([-A-1,A+1,0-1,B+1]) % Dimensión del espacio

%% Plot de estanterías

rectangle('Position',[1 2 5 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[1 5 5 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[1 8 5 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[1 11 5 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[-5 2 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[-5 5 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[-5 8 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[-5 11 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[8 2 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[8 5 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[8 8 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[8 11 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])

rectangle('Position',[-5 14 5 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[0 14 1 3],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[6 14 4 1],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])
rectangle('Position',[6 14 1 3],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])

rectangle('Position',[3 14 1 3],'FaceColor',[0.5,0.5,0.5],'EdgeColor',[0.5,0.5,0.5])

%bordes del mapa
line([-12,12],[0,0],'Color',[0.5,0.5,0.5],'linewidth',3)
line([12,12],[0,17],'Color',[0.5,0.5,0.5],'linewidth',3)
line([12,-12],[17,17],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-12,-12],[17,0],'Color',[0.5,0.5,0.5],'linewidth',3)

line([-5,-5],[0,12],'Color',[0.5,0.5,0.5],'linewidth',3)
line([3.5,3.5],[0,2],'Color',[0.5,0.5,0.5],'linewidth',3)
line([3.5,3.5],[6,8],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-5,-10],[2.5,2.5],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-5,-7],[5.5,5.5],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-12,-9],[5.5,5.5],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-5,-9],[8.5,8.5],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-9,-9],[8.5,15],'Color',[0.5,0.5,0.5],'linewidth',3)
line([-9,-7],[15,15],'Color',[0.5,0.5,0.5],'linewidth',3)

for i=1:size(t,1)
plot(t(i,1),t(i,2),'.','markersize',15,'color','r')    
end

%% Inicialización

% Iteraciones máximas del algoritmo
maxIterations=10000;

% Incremento de segmentos
inc=0.2;

% Nodo goal (comprobar que el nodo goal es accesible)
contadorIntentoGoal=0;
pruebaGoal=10;  % Número de iteraciones antes de comprobar goal
encontrado=0;
plot(xg,yg,'bo');

% Nodo inicial
plot(xo,yo,'ro');

% Estructura del árbol
arbol=struct;
arbol.x(1)=xo;
arbol.y(1)=yo;
arbol.p(1)=0;     % Padre (0 caso base)

%% Algoritmo

if choque(xg,yg)==1
    
    for bucle=0:maxIterations
        if contadorIntentoGoal==pruebaGoal
            rx=xg;
            ry=yg;
            contadorIntentoGoal=0;
        else
            % Elección de nodo nuevo aleatorio
            bool=0;
            while bool==0
                rx=-A+(A*2)*rand();
                ry=B*rand();
                bool=choque(rx,ry);
            end
        end

        contadorIntentoGoal=contadorIntentoGoal+1;
        % Encontrar nodo más cercano del árbol a nodo random
        cercano=[0,0];
        [cercano(1),cercano(2),indicePadre]=nearest(rx,ry,arbol);
        x=cercano(1);
        y=cercano(2);
        
        % Segmentación y coordenadas de puntos intermedios
        segmentos=floor(sqrt((x-rx)^2+(y-ry)^2)/inc);
        
        if sqrt((x-rx)^2+(y-ry)^2)<inc
           segmentos=2;
        end
        
        segX=linspace(x,rx,segmentos+1);
        segY=linspace(y,ry,segmentos+1);
        puntosIntermediosCandidatos=[segX;segY];
        
        
        
        puntosIntermedios=[]; %Inicialización vacía para plot de nodos nuevos del tree
        
        for i=1:size(puntosIntermediosCandidatos,2) %comprueba choque
            bool=choque(puntosIntermediosCandidatos(1,i),puntosIntermediosCandidatos(2,i));
            if bool==1  % Si no choca
                % Añade puntos válidos a vector temporal para plot
                puntosIntermedios(1,size(puntosIntermedios,2)+1)=puntosIntermediosCandidatos(1,i);
                puntosIntermedios(2,size(puntosIntermedios,2))=puntosIntermediosCandidatos(2,i);
                %Añade puntos válidos a lista de nodos general
                if i~=1
                    arbol.x(length(arbol.x)+1)=puntosIntermediosCandidatos(1,i);
                    arbol.y(length(arbol.x))=puntosIntermediosCandidatos(2,i);
                    if i==2
                        arbol.p(length(arbol.x))=indicePadre;
                    else
                        arbol.p(length(arbol.x))=length(arbol.x)-1;
                    end
                end
                if xg==puntosIntermediosCandidatos(1,i) && yg==puntosIntermediosCandidatos(2,i)
                    encontrado=1;
                    break
                end
            elseif bool==0
                break
            end
        end
        % Dibuja la nueva rama solo si esta existe
        if ~isempty(puntosIntermedios)
            plot(puntosIntermedios(1,:),puntosIntermedios(2,:),'color','k')
            pause(0.01)
        end
        
        if encontrado==1
            disp('Goal encontrado.')
            hilo=backtracking_RRT(arbol,xo,yo);
            plot(hilo(1,:),hilo(2,:),'Linewidth',1,'color','b')
            break
        end
    end % End del bucle RRT.
    
else
    
    disp('Goal fuera del mapa.')
    hilo=[0;0];
    
end % End de condición de goal fuera del mapa.

if encontrado==0
    disp('Goal no encontrado.')
end
end