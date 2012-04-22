function control

% ode45 solver
step=0.01;
t=0:step:10;
initial_quarter=[0; 0; 0; 0;]; % 2dof * 2
initial_half=[0; 0; 0; 0; 0; 0; 0; 0;]; % 4dof * 4
initial_full=[0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;]; % 7dof * 2
[t,x]=ode45(@quarter_car,[0 5],initial_quarter);
%[t,x]=ode45(@half_car,t,initial_half);
%[t,x]=ode45(@full_car,t,initial_full);

% plot
subplot(2,4,1);
plot(t,x(:,1));
xlabel('x_s (offset of body)');

subplot(2,4,2);
plot(t,x(:,2));
xlabel('x_u (offset of tire)');

subplot(2,4,3);
plot(t,x(:,3));
xlabel('v_s (velocity of body)');

subplot(2,4,4);
plot(t,x(:,4));
xlabel('v_u (velocity of tire)');

% subplot(2,4,5);
% plot(t,x(:,5));
% 
% subplot(2,4,6);
% plot(t,x(:,6));
% 
% subplot(2,4,7);
% plot(t,x(:,7));
% 
% subplot(2,4,8);
% plot(t,x(:,8));

subplot(2,4,5);
plot(t,x(:,1)+x(:,2));
xlabel('Body position from ground');

subplot(2,4,6);
plot(t,x(:,3)+x(:,4));
xlabel('Body velocity from ground');


%axis([0 10 0 0.02]);

    % Ground function
    function out=F1(t)
        %force_input=0.01*ku/mu;
        force_input=0.1;
        if t<1
            out=0;
        else
            out=force_input;
        end
        
        out=0;
    end

    % Quarter-car
    function dxdt=quarter_car(t,x)
%         % Quarter-car Masses
         ms=375;     % Body mass
         mu=75;      % Tire mass
%         % Springs
         ks=35000;   % Shocks
         ku=193000;  % Tire
%         % Dampers
         bs=2000;    % Shocks
         bu=0;       % Tire
        % Masses
        %ms=500;     % Body mass
        %mu=7;      % Tire mass
        % Springs
        %ks=1;   % Shocks
        %ku=;  % Tire
        % Dampers
       % bs=1;    % Shocks
       % bu=1;       % Tire
        
        dxdt=zeros(size(x));
        
        m=[ms 0; 0 mu;];
        b=[bs -bs; -bs (bs+bu);];
        k=[ks -ks; -ks (ks+ku);];
        F=[0-ms*9.81; 0;];
        %F = [-ms*9.81; 0];
        %F=[0; 0;];
        %g=[ms*9.81; (ms+mu)*9.81;];
        
        % x(1) = x of car
        % x(2) = x of tire
        % x(3) = vel of car
        % x(4) = vel of tire
        
        dxdt2=inv(m)*(F-b*[x(3); x(4);]-k*[x(1); x(2);]); % '\'==fast inverse
        dxdt=[x(3); x(4); dxdt2(1); dxdt2(2);];
    end

    % Half-car
    function dxdt=half_car(t,x)
        % Half-car Masses
        mu=840/2;    % Body mass
        m1=53;      % Tire mass
        m2=53;      % Tire mass
        Ix=820;     % Body inertia
        % Body lengths
        b1=0.7;
        b2=0.75;
        % Springs
        ku=10000;
        kt=200000;
        % Dampers
        bu=2000;     % Shocks

        dxdt=zeros(size(x));
        state_x_dot=x(1:4);
        state_x=x(5:8);

        m=[mu 0 0 0;
           0 Ix 0 0;
           0 0 m1 0;
           0 0 0 m2;];
        b=[2*bu (bu*b1-bu*b2) -bu -bu;
           (bu*b1-bu*b2) (bu*b1^2+bu*b2^2) -bu*b1 bu*b2;
           -bu -bu*b1 bu 0;
           -bu bu*b2 0 bu;];
        k=[2*ku (ku*b1-ku*b2) -ku -ku;
           (ku*b1-ku*b2) (ku*b1^2+ku*b2^2) -ku*b1 ku*b2;
           -ku -ku*b1 (ku+kt) 0;
           -ku ku*b2 0 (ku+kt);];
        F=[0; 0; F1(t)*kt; F1(t)*kt;];
        g=[9.81; 0; 9.81; 9.81;];
        
        dxdt2=m\(F-b*state_x_dot-k*state_x-g);
        
        dxdt(1)=x(5);
        dxdt(2)=x(6);
        dxdt(3)=x(7);
        dxdt(4)=x(8);
        dxdt(5)=dxdt2(1);
        dxdt(6)=dxdt2(2);
        dxdt(7)=dxdt2(3);
        dxdt(8)=dxdt2(4);
    end

    % Full-car
    function dxdt=full_car(t,x)
        mu=840;
        mf=53;
        mr=76;
        Ix=820;
        Iy=1100;
        a1=1.4;
        a2=1.47;
        b1=0.7;
        b2=0.75;
        w=b1+b2;
        kf=10000;
        kr=13000;
        ktf=200000;
        ktr=200000;
        kR=0;
        cf=2000;
        cr=2000;
        
        dxdt=zeros(size(x));
        state_x_dot=x(1:7);
        state_x=x(8:14);
        
        m=[mu 0 0 0 0 0 0;
           0 Ix 0 0 0 0 0;
           0 0 Iy 0 0 0 0;
           0 0 0 mf 0 0 0;
           0 0 0 0 mf 0 0;
           0 0 0 0 0 mr 0;
           0 0 0 0 0 0 mr;];
       
        % Damping variables
        c11=2*cf+2*cr;
        c12=b1*cf-b2*cf-b1*cr+b2*cr;
        c21=c12;
        c13=2*a2*cr-2*a1*cf;
        c31=c13;
        c22=b1^2*cf+b2^2*cf+b1^1*cr+b2^2*cr;
        c23=a1*b2*cf-a1*b1*cf-a2*b1*cr+a2*b2*cr;
        c32=c23;
        c33=2*cf*a1^2+2*cr*a2^2;
        b=[c11 c12 c13 -cf -cf -cr -cr;
           c21 c22 c23 -b1*cf b2*cf b1*cr -b2*cr;
           c31 c32 c33 a1*cf a1*cf -a2*cr -a2*cr;
           -cf -b1*cf a1*cf cf 0 0 0;
           -cf b2*cf a1*cf 0 cf 0 0;
           -cr b1*cr -a2*cr 0 0 cr 0;
           -cr -b2*cr -a2*cr 0 0 0 cr;];
        
        % Spring variables
        k11=2*kf+2*kr;
        k12=b1*kf-b2*kf-b1*kr+b2*kr;
        k21=k12;
        k13=2*a2*kr-2*a1*kf;
        k31=k13;
        k22=kR+b1^2*kf+b2^2*kf+b1^2*kr+b2^2*kr;
        k23=a1*b2*kf-a1*b1*kf-a2*b1*kr+a2*b2*kr;
        k32=k23;
        k24=-b1*kf-1/w*kR;
        k42=k24;
        k25=b2*kf+1/w*kR;
        k52=k25;
        k33=2*kf*a1^2+2*kr*a2^2;
        k44=kf+ktf+1/w^2*kR;
        k55=kf+ktf+1/w^2*kR;
        k=[k11 k12 k13 -kf -kf -kr -kr;
           k21 k22 k23 k24 k25 b1*kr -b2*kr;
           k31 k32 k33 a1*kf a1*kf -a2*kr -a2*kr;
           -kf k42 a1*kf k44 -kR/w^2 0 0;
           -kf k52 a1*kf -kR/w^2 k55 0 0;
           -kr b1*kr -a2*kr 0 0 (kr+ktr) 0;
           -kr -b2*kr -a2*kr 0 0 0 (kr+ktr);];

        F=[0; 0; 0; F1(t)*ktf; F1(t)*ktf; F1(t)*ktf; F1(t)*ktf;];
        
        dxdt2=m\(F-b*state_x_dot-k*state_x);

        dxdt(1)=x(8);
        dxdt(2)=x(9);
        dxdt(3)=x(10);
        dxdt(4)=x(11);
        dxdt(5)=x(12);
        dxdt(6)=x(13);
        dxdt(7)=x(14);
        dxdt(8)=dxdt2(1);
        dxdt(9)=dxdt2(2);
        dxdt(10)=dxdt2(3);
        dxdt(11)=dxdt2(4);
        dxdt(12)=dxdt2(5);
        dxdt(13)=dxdt2(6);
        dxdt(14)=dxdt2(7);
    end
end