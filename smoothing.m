function [out]=smoothing(hilo,inc)

% Recibiendo un vector de puntos (x,y) y un valor de incremento,
% realiza 500 iteraciones eligiendo dos puntos aleatorios del vector
% de puntos de entrada e intenta unirlos mediante una recta. Esta 
% recta se discretiza utilizando según el incremento dado. Si en ese 
% intento de unión, algún punto creado choca (según la función anterior),
% se aborta el intento de unión y se pasa a la siguiente iteración, 
% probando con otros dos puntos aleatorios del vector. En el caso de 
% que no choque ningún punto en el intento de unión, se reemplazarán 
% los puntos intermedios del vector por los nuevos de la recta creada.

for i=1:500
    
    indiceinicio=0; % Nodo donde empieza la recta.
    indicefinal=0;  % Nodo de fin de recta.
    
    % Evita que se redondee a un índice 0.
    while indiceinicio==0
        indiceinicio=round(rand()*size(hilo,2));
    end
    % Evita que se redondee a un índice 0.
    while indicefinal==0
        indicefinal=round(rand()*size(hilo,2));
    end
    
    if indicefinal<indiceinicio
        asd=indicefinal;
        indicefinal=indiceinicio;
        indiceinicio=asd;
    end
    
    xini=hilo(1,indiceinicio);
    yini=hilo(2,indiceinicio);
    
    xfin=hilo(1,indicefinal);
    yfin=hilo(2,indicefinal);
    
    % Segmenta la recta que une los dos puntos en función del incremento.
    segmentos=floor(sqrt((xini-xfin)^2+(yini-yfin)^2)/inc);
    segX=linspace(xini,xfin,segmentos+1);
    segY=linspace(yini,yfin,segmentos+1);
    puntosIntermediosCandidatos=[segX;segY];
    
    puntosIntermedios=[]; %Inicialización vacía para plot de nodos nuevos del tree
    
    for i=1:size(puntosIntermediosCandidatos,2) %comprueba choque
        bool=choque(puntosIntermediosCandidatos(1,i),puntosIntermediosCandidatos(2,i));
        if bool==1
            puntosIntermedios(1,size(puntosIntermedios,2)+1)=puntosIntermediosCandidatos(1,i);
            puntosIntermedios(2,size(puntosIntermedios,2))=puntosIntermediosCandidatos(2,i);
            if i==size(puntosIntermediosCandidatos,2)
                out=hilo(:,1:indiceinicio-1);
                out=[out puntosIntermedios hilo(:,indicefinal+1:size(hilo,2))];
                hilo=out;
            end
        elseif bool==0 % Si algún punto de la recta choca, termina el intento.
            break
        end
    end
end

plot(hilo(1,:),hilo(2,:),'r','linewidth',1);

end
