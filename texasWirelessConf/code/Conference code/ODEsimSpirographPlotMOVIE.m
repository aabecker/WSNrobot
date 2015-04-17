function ODEsimSpirographPlotMOVIE
% makes a pretty MOVIE of many robots being driven by a constantly rotating
% magnetic field.
%
% Author: Aaron Becker, aabecker@gmail.com
% Date: 5/14/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global  freq vel a
set(0,'defaultaxesfontsize',14);
set(0,'defaulttextfontsize',14);
set(0,'DefaultTextInterpreter', 'latex')
format compact
clc



freq = 4;%10;  % as the frequency increases, the lines become straighter, but the differentiation decreases)
vel = 0.7; %mm/s (Estimate by Yan Ou) 700 \mum/s

tF = 18;  %final time

%initial conditions for cells (x,t,theta)
[y,x] = meshgrid(1:2,1:4);
x = reshape(x,numel(x),1);
y = 3-reshape(y,numel(y),1);
N = numel(x); %number of robots
a = linspace(4,11,N)'; % a values, should be between 3 and 10
th = 0*ones(N,1); % performance is very dependent on initial orientation for large amplitude

options = odeset('RelTol',1e-6,'AbsTol',1e-6 );
[T,Y] = ode45(@simPyriformis,[0:1/60:tF],[x;y;th],options);


%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%draw path, cells
figure(1)
clf
c = distinguishable_colors(N+1);
for i = 1:N
    
    s = plot(Y(1,i),Y(1,i+N),'x','color',c(i,:));
    uistack(s,'bottom');
    hold on
    p = plot(Y(:,i),Y(:,i+N),'-','color',c(i,:));
    uistack(p,'bottom');
    text(Y(1,i),Y(1,i+N)-.2,['$a$=',num2str(a(i))],'HorizontalAlignment','center','Interpreter','latex')
end
axis equal
title({[num2str(N),' cells simulated for ',num2str(tF),' s'];},'Interpreter','latex');%['\theta_M = ',num2str(Mag),'sin(',num2str(freq),'t)']});
xlabel('$x$ mm','Interpreter','latex')
ylabel('$y$ mm','Interpreter','latex')
for i = 1:N
    drawCell(Y(end,i),Y(end,i+N),Y(end,i+2*N),a(i),0.05);
end

%fix the axis and xticks
set(gca, 'xtick',[-5:10],'ytick',[-5:10])
axis([0,4.5,.5,3.5])

%saveas(gcf,'../../pictures/pdf/SpiralsFromRotatingMagneticField.pdf')
format_ticks(gca,{'0','1','2','3','4'},{'1','2','3'},[0:4],[1:3],0,0,0.01);
xlabel({' ';'$x$ mm'},'Interpreter','latex')
ylabel({'$y$ mm';' '},'Interpreter','latex')
set(gcf,'PaperUnits','inches')
set(gcf,'papersize',[7,4])
set(gcf,'paperposition',[-.5,-.5,8,5])

%print -dpdf '../../pictures/pdf/SpiralsFromRotatingMagneticField.pdf'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MAKE_MOVIE = true;
if MAKE_MOVIE
    clf
    set(0,'defaultaxesfontsize',24);
set(0,'defaulttextfontsize',24);
set(0,'DefaultTextInterpreter', 'latex')
    
    
    figure(2)
    
    
    FrameCount = 1;
    MOVIE_NAME = 'SpirographV3';
    G.fig = figure(2);
    clf
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(writerObj,'Quality',100)
    %set(writerObj, 'CompressionRatio', 2);
    open(writerObj);
    
    %Init plots
    hPath = zeros(N,1);
    hPathMag = zeros(N,1);
    hPathVel = zeros(N,1);
    
   
        arrX = 0.5*[0,0,    0.4,0.4,  0.7,   0.4, 0.4, 0,   0];
    arrY = 0.5*[0,0.1,  0.1,0.2,  0,  -0.2,-0.1,-0.1, 0];
    asc = 0.2;
    ascx = 0.5;
    phiT = calcPhi(T(i));
    for i = 1:N
        hPath(i) = plot(Y(1,i),Y(1,i+N),'-','color',c(i+1,:), 'LineSmoothing','on');
        set(hPath(i),'linewidth',3);
        hold on
        %uistack(hPath(i),'bottom');
        hold on
        hPathMag(i) = patch(Y(1,i)+ cos(phiT)*ascx*arrX - sin(phiT)*asc*arrY,Y(1,i+N)+ sin(phiT)*ascx*arrX + cos(phiT)*asc*arrY,-0.5*ones(size(arrY)),'b', 'LineSmoothing','on');
        hPathVel(i) = patch(Y(1,i)+ cos(Y(1,i+2*N))*ascx*arrX - sin(Y(1,i+2*N))*asc*arrY,Y(1,i+N)+ sin(Y(1,i+2*N))*ascx*arrX + cos(Y(1,i+2*N))*asc*arrY,-0.2*ones(size(arrY)),c(i+1,:), 'LineSmoothing','on');
               
        text(Y(1,i),Y(1,i+N)-.2,1,['$a$=',num2str(a(i))],'HorizontalAlignment','center','Interpreter','latex')
    end
    set(gca, 'xtick',[-5:10],'ytick',[-5:10])
axis([0,4.5,.5,3.5])
    axis equal
    
    hCells = cell(N,1);
    for j = 1:N
           hCells{j} =  drawCell(Y(1,j),Y(1,j+N),Y(1,j+2*N),a(j),0.05);
    end

    hArr = patch(1.55+ arrX,1.55+ arrY,-1*ones(size(arrY)),'b', 'LineSmoothing','on');
    
    hold on
    %text(2.2,1.55,'Magnetic Field, \phi=','Color','b','HorizontalAlignment','center','Interpreter','latex');
    %hMag = text(2,1.55,['Magnetic Field, $\phi=$',num2str(mod(round(phiT*180/pi),360)),'$^{\circ}$'],'Color','b','HorizontalAlignment','left','Interpreter','latex');
    hMag = text(2,1.55,1,['Magnetic Field, $f=$',num2str(round(calcFreq(T(i))*10)/10),'Hz'],'Color','b','HorizontalAlignment','left','Interpreter','latex');
    
    hold on
    uistack(hArr,'bottom')
    
    %fix the axis and xticks
    set(gca, 'xtick',[-5:10],'ytick',[-5:10])
    axis([0,4.5,.5,3.5])

    format_ticks(gca,{'0','1','2','3','4'},{'1','2','3'},[0:4],[1:3],0,0,0.01);
    xlabel({' ';'$x$ mm'},'Interpreter','latex')
    ylabel({'$y$ mm';' '},'Interpreter','latex')
    
    % UPDATE PLOTS
    for i = 1:2:length(T)
        
        phiT = calcPhi(T(i));
        for j = 1:N
            updateCell(hCells{j}, Y(i,j),Y(i,j+N),Y(i,j+2*N));
            set(hPath(j),'Xdata',Y(1:i,j),'Ydata',Y(1:i,j+N),'Zdata',-2*ones(size(Y(1:i,j))));
            
            set(hPathMag(j), 'Xdata',Y(i,j)+ cos(phiT)*ascx*arrX - sin(phiT)*asc*arrY, 'Ydata',Y(i,j+N)+ sin(phiT)*ascx*arrX + cos(phiT)*asc*arrY);
            set(hPathVel(j), 'Xdata',Y(i,j)+ cos(Y(i,j+2*N))*ascx*arrX - sin(Y(i,j+2*N))*asc*arrY, 'Ydata',Y(i,j+N)+ sin(Y(i,j+2*N))*ascx*arrX + cos(Y(i,j+2*N))*asc*arrY);
            
        end
        
        set(hMag,'String',['Magnetic Field, $f=$',num2str(round(calcFreq(T(i))*10)/10),'Hz']);
%         if T(i)<5
%         set(hMag,'String',['Magnetic Field, $\phi=$',num2str(mod(round(phiT*180/pi),360)),'$^{\circ}$']);
%         else
%           set(hMag,'String','');
%         end
        set(hArr, 'Xdata',1.55+ cos(phiT)*arrX - sin(phiT)*arrY, 'Ydata',1.55+ sin(phiT)*arrX + cos(phiT)*arrY);
        %drawnow
        updateDrawing
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% END MAKE A MOVIE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function f =calcFreq(t)
        % 5 seconds at 4 Hz, ramp up to 10 Hz, for 3 seconds, and 10 seconds at 10 Hz.
        if t<5
            f = 4;
        elseif t<8
            f = 4+(t-5)*2;
        else
            f = 10;
        end
    end

    function phi =calcPhi(t)
        % 5 seconds at 4 Hz, ramp up to 10 Hz, for 3 seconds, and 10 seconds at 10 Hz.
        if t<5
            phi = t*4;
        elseif t<8
            phi = t.^2-6*t+25;
        else
            phi = 41+10*(t-8);
        end
    end

    function dq = simPyriformis(t,q)
        % simulates a differential equation model for the cells
        % q = [x_coordinates; y_coordinates; theta_coordinates,
        % magnetic_field_orientation]

        dq = zeros(size(q));    % a column vector for the change in state
        N = (numel(q))/3;  %number of robots
        th = q(2*N+1:3*N); %current orientations of the cells
        
        phi = calcPhi(t);
        
        dq(1:N)       = vel*cos(th);
        dq(N+1:2*N)   = vel*sin(th);
        angDif = phi-th;
        dq(2*N+1:3*N) = a.*sin(angDif);
        
    end


    function updateDrawing
        drawnow
        if(MAKE_MOVIE)
            FrameCount=FrameCount+1;
            figure(G.fig)
            set(gcf,'renderer','painters')
            tfig = myaa(3);
            %F = getframe_nosteal_focus; %
            F = getframe;
            writeVideo(writerObj,F.cdata);
            close(tfig)
            while(FrameCount < 10)
                updateDrawing
            end
            
        end
    end
end
