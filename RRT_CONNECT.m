function [hilo]=RRT_CONNECT()


% Se basa en la misma lógica que el algoritmo RRT, pero utilizando dos árboles,
% uno nace del nodo origen y otro nace del nodo destino. Intentan encontrarse 
% según la siguiente secuencia, que se repite un numero configurado de veces 
% hasta que los árboles consigan unirse al mismo punto.
% •	Árbol 1(origen) intenta unirse a un punto aleatorio.
% •	Árbol 2 (destino) intenta unirse al último punto añadido al árbol 1.
% •	Árbol 2 intenta unirse a un punto aleatorio.
% •	Árbol 1 intenta unirse al último punto añadido al árbol 2.
% Una vez ambos arboles se encuentran, mediante la función backtracking.m 
% se encuentran su vector solución, y estos se concatenan de forma ordenada,
% devolviendo una única matriz de puntos solución.




global robot world A B xg yg xo yo t


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
    plot(t(i,1),t(i,2),'.','markersize',10,'color','r')
end

%% Inicialización

% Iteraciones máximas del algoritmo
maxIterations=10000;

% Incremento de segmentos
inc=0.2;

% Nodo inicial
inicial=apoloGetLocationMRobot(robot,world);
xo=inicial(1);
yo=inicial(2);

plot(xo,yo,'ro');
plot(xg,yg,'bo');

encontrado=0;

% Estructura del árbol 1 y 2
arbol1=struct;
arbol1.x(1)=xo;
arbol1.y(1)=yo;
arbol1.p(1)=0;     % Padre (0 caso base)

arbol2=struct;
arbol2.x(1)=xg;
arbol2.y(1)=yg;
arbol2.p(1)=0;     % Padre (0 caso base)

%% Algoritmo

if choque(xg,yg)==1 %comprueba si el nodo goal esta dentro del mapa
    
    for bucle=0:maxIterations %inicia el bucle
        
        %% Parte 1: El arbol 1 intenta unirse a un punto aleatorio
        
        % Elección de nodo nuevo aleatorio
        bool=0;
        while bool==0
            
            rx=-A+(A*2)*rand();
            ry=B*rand();
            %comprueba que el punto aleatorio sea válido
            bool=choque(rx,ry);
        end
        
        % Encontrar nodo más cercano del árbol a nodo random
        cercano=[0,0];
        [cercano(1),cercano(2),indicePadre]=nearest(rx,ry,arbol1);
        x=cercano(1);
        y=cercano(2);
        
        % Segmentación y coordenadas de puntos intermedios
        segmentos=floor(sqrt((x-rx)^2+(y-ry)^2)/inc);
        segX=linspace(x,rx,segmentos+1);
        segY=linspace(y,ry,segmentos+1);
        puntosIntermediosCandidatos=[segX;segY];
        puntosIntermedios=[]; %Inicialización vacía para plot de nodos nuevos del tree
        
        for i=1:size(puntosIntermediosCandidatos,2) %comprueba choque
            bool=choque(puntosIntermediosCandidatos(1,i),puntosIntermediosCandidatos(2,i));
            if bool==1  % Si no choca el punto intermedio
                % Añade puntos válidos a vector temporal para plot
                puntosIntermedios(1,size(puntosIntermedios,2)+1)=puntosIntermediosCandidatos(1,i);
                puntosIntermedios(2,size(puntosIntermedios,2))=puntosIntermediosCandidatos(2,i);
                %Añade puntos válidos a lista de nodos general
                if i~=1
                    %como no choca, lo añade al arbol
                    arbol1.x(length(arbol1.x)+1)=puntosIntermediosCandidatos(1,i);
                    arbol1.y(length(arbol1.x))=puntosIntermediosCandidatos(2,i);
                    if i==2
                        arbol1.p(length(arbol1.x))=indicePadre;
                    else
                        %va guardando los padres
                        arbol1.p(length(arbol1.x))=length(arbol1.x)-1;
                    end
                end
                
            elseif bool==0 %si el punto choca, aborta
                break
            end
            
        end
        
        % Dibuja la nueva rama solo si esta existe
        if ~isempty(puntosIntermedios)
            plot(puntosIntermedios(1,:),puntosIntermedios(2,:),'color','k')
            pause(0.001)
        end
        
        %% Parte 2: El arbol 2 intenta unirse al ultimo punto del arbol 1
        
        %puntos objetivo_ ultimo nodo de arbol 1
        rx=arbol1.x(length(arbol1.x));
        ry=arbol1.y(length(arbol1.y));
        
        % Encontrar nodo más cercano del árbol al nodo objetivo
        cercano2=[0,0];
        [cercano2(1),cercano2(2),indicePadre2]=nearest(rx,ry,arbol2);
        x2=cercano2(1);
        y2=cercano2(2);
        
        % Segmentación y coordenadas de puntos intermedios
        segmentos2=floor(sqrt((x2-rx)^2+(y2-ry)^2)/inc);
        segX2=linspace(x2,rx,segmentos2+1);
        segY2=linspace(y2,ry,segmentos2+1);
        puntosIntermediosCandidatos2=[segX2;segY2];
        puntosIntermedios2=[]; %Inicialización vacía para plot de nodos nuevos del tree
        
        for i=1:size(puntosIntermediosCandidatos2,2) %comprueba choque
            bool2=choque(puntosIntermediosCandidatos2(1,i),puntosIntermediosCandidatos2(2,i));
            if bool2==1  % Si no choca el punto intermedio
                
                % Añade puntos válidos a vector temporal para plot
                puntosIntermedios2(1,size(puntosIntermedios2,2)+1)=puntosIntermediosCandidatos2(1,i);
                puntosIntermedios2(2,size(puntosIntermedios2,2))=puntosIntermediosCandidatos2(2,i);
                %Añade puntos válidos a lista de nodos general
                if i~=1
                    arbol2.x(length(arbol2.x)+1)=puntosIntermediosCandidatos2(1,i);
                    arbol2.y(length(arbol2.x))=puntosIntermediosCandidatos2(2,i);
                    if i==2
                        arbol2.p(length(arbol2.x))=indicePadre2;
                    else
                        arbol2.p(length(arbol2.x))=length(arbol2.x)-1;
                    end
                end
                
            elseif bool2==0 %si choca el punto intermedio, aborta
                break
            end
        end
        
        % Dibuja la nueva rama solo si esta existe
        if ~isempty(puntosIntermedios2)
            plot(puntosIntermedios2(1,:),puntosIntermedios2(2,:),'color','k')
            pause(0.001)
        end
        %comprueba si ambos arboles se han encontrado, lo que
        %terminaria el proceso
        if arbol1.x(length(arbol1.x))==arbol2.x(length(arbol2.x)) && arbol1.y(length(arbol1.x))==arbol2.y(length(arbol2.x))
            encontrado=1;
            break
        end
        
        %% Parte 3: El arbol 3 intenta unirse a un nuevo punto aleatorio

        bool=0;
        while bool==0
            
            rx=-A+(A*2)*rand();
            ry=B*rand();
            %comprueba que el punto aleatorio sea válido
            bool=choque(rx,ry);
        end
        
        % Encontrar nodo más cercano del árbol a nodo random
        cercano2=[0,0];
        [cercano2(1),cercano2(2),indicePadre2]=nearest(rx,ry,arbol2);
        x2=cercano2(1);
        y2=cercano2(2);
        
        % Segmentación y coordenadas de puntos intermedios
        segmentos2=floor(sqrt((x2-rx)^2+(y2-ry)^2)/inc);
        segX2=linspace(x2,rx,segmentos2+1);
        segY2=linspace(y2,ry,segmentos2+1);
        puntosIntermediosCandidatos2=[segX2;segY2];
        puntosIntermedios2=[]; %Inicialización vacía para plot de nodos nuevos del tree
        
        for i=1:size(puntosIntermediosCandidatos2,2) %comprueba choque
            bool2=choque(puntosIntermediosCandidatos2(1,i),puntosIntermediosCandidatos2(2,i));
            if bool2==1  % Si no choca
                
                % Añade puntos válidos a vector temporal para plot
                puntosIntermedios2(1,size(puntosIntermedios2,2)+1)=puntosIntermediosCandidatos2(1,i);
                puntosIntermedios2(2,size(puntosIntermedios2,2))=puntosIntermediosCandidatos2(2,i);
                %Añade puntos válidos a lista de nodos general
                if i~=1
                    arbol2.x(length(arbol2.x)+1)=puntosIntermediosCandidatos2(1,i);
                    arbol2.y(length(arbol2.x))=puntosIntermediosCandidatos2(2,i);
                    if i==2
                        arbol2.p(length(arbol2.x))=indicePadre2;
                    else
                        arbol2.p(length(arbol2.x))=length(arbol2.x)-1;
                    end
                end
                
            elseif bool2==0 %si el punto choca, se aborta.
                break
            end
        end
        
        % Dibuja la nueva rama solo si esta existe
        if ~isempty(puntosIntermedios2)
            plot(puntosIntermedios2(1,:),puntosIntermedios2(2,:),'color','k')
            pause(0.001)
        end
        
        %% Parte 4: El arbol 1 intenta unirse al ultimo punto del arbol 2
        
        %puntos objetivo_ ultimo nodo de arbol 2
        rx=arbol2.x(length(arbol2.x));
        ry=arbol2.y(length(arbol2.y));
        
        % Encontrar nodo más cercano del árbol a nodo objetivo
        cercano=[0,0];
        [cercano(1),cercano(2),indicePadre]=nearest(rx,ry,arbol1);
        x=cercano(1);
        y=cercano(2);
        
        % Segmentación y coordenadas de puntos intermedios
        segmentos=floor(sqrt((x-rx)^2+(y-ry)^2)/inc);
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
                    arbol1.x(length(arbol1.x)+1)=puntosIntermediosCandidatos(1,i);
                    arbol1.y(length(arbol1.x))=puntosIntermediosCandidatos(2,i);
                    if i==2
                        arbol1.p(length(arbol1.x))=indicePadre;
                    else
                        arbol1.p(length(arbol1.x))=length(arbol1.x)-1;
                    end
                end
                
            elseif bool==0 %si el punto choca, se aborta.
                break
            end
        end
        
        % Dibuja la nueva rama solo si esta existe
        if ~isempty(puntosIntermedios)
            plot(puntosIntermedios(1,:),puntosIntermedios(2,:),'color','k')
            pause(0.001)
        end
        %comprueba si los arboles se han encontrado, que seria
        %solucion
        if arbol1.x(length(arbol1.x))==arbol2.x(length(arbol2.x)) && arbol1.y(length(arbol1.x))==arbol2.y(length(arbol2.x))
            encontrado=1;
            break
        end
        
    end
    
else %si el goal esta fuera del mapa
    
    disp('Goal fuera del mapa')
    hilo=[0;0];
end

% Informacion para el usuario
if encontrado==0
    disp('Goal no encontrado.')
else
    disp('Goal encontrado')
    
    %% Creación del hilo de solución
    
    % Parte del arbol 1
    hilo=backtracking_RRTCONNECT(arbol1,xo,yo);
    camino=[1;1];
    for i=1:size(hilo,2)
        camino(1,i)=hilo(1,size(hilo,2)+1-i);
        camino(2,i)=hilo(2,size(hilo,2)+1-i);
    end
    
    % Parte del arbol 2
    hilo2=backtracking_RRTCONNECT(arbol2,xg,yg);
    
    % Unión de los hilos
    hilo=[camino hilo2];
    
    % Plotea el hilo solucion
    plot(hilo(1,:),hilo(2,:),'Linewidth',1,'color','b')

end
end
