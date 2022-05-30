close all
clear all
clc

h = 0.1;
t = (0:h:80)';
%счетные константы
n = length(t);

c = [0.5, 0.25]; %минимальный угол поворота руля
U0 = [5; -10]; %ну
U = randn_U(c,U0,n);

%первый корабль
%структура коэф kof={Tr Kr Kd Td P}
kof(1) = const2struc(20, 0.8, 2, 2, 5);
%начальные условия
F10 = [deg2rad(5) 0 0 0 0];% delta, psi, dpsi, x,y
F1 = zeros(n,5);
F1(1,:) = F10; 
%объект корабль 1
sh1 = ship1(F1, kof(1), U(1,:));

%второй корабль
%структура коэф kof={Tr Kr Kd Td P}
kof(2) = const2struc(20, 0.9, 0.9, 6, -10);
%начальные условия
F20 = [deg2rad(20)  deg2rad(90) deg2rad(2) 20 15];% delta, psi, dpsi, x,y
F2 = zeros(n,5);
F2(1,:) = F20;
%объект корабль 2
sh2 = ship1(F2, kof(2), U(2,:));


% координаты Х
X = zeros(n,2);
% координаты Y и скорость
Y = zeros(n,2);

%F в объекте должен быть матрицей или вектором ну?
for i= 1 : n-1
    %РК4
    k1 = sh1.fnc_dF(t(i), sh1.F(i,:),i);
    k2 = sh1.fnc_dF(t(i)+0.5*h, sh1.F(i,:)+0.5*h*k1,i);
    k3 = sh1.fnc_dF(t(i)+0.5*h, sh1.F(i,:)+0.5*h*k2,i);
    k4 = sh1.fnc_dF(t(i)+h, sh1.F(i,:)+h*k3,i);
    sh1.F(i+1,:) = sh1.F(i,:) + h/6*(k1 + 2 * k2 + 2 * k3 + k4);

    l1 = sh2.fnc_dF(t(i), sh2.F(i,:),i);
    l2 = sh2.fnc_dF(t(i)+0.5*h, sh2.F(i,:)+0.5*h*l1,i);
    l3 = sh2.fnc_dF(t(i)+0.5*h, sh2.F(i,:)+0.5*h*l2,i);
    l4 = sh2.fnc_dF(t(i)+h, sh2.F(i,:)+h*l3,i);
    sh2.F(i+1,:) = sh2.F(i,:) + h/6*(l1 + 2 * l2 + 2 * l3 + l4);
end

X(:,1) = sh1.F(:,4);
Y(:,1) = sh1.F(:,5);
X(:,2) = sh2.F(:,4);
Y(:,2) = sh2.F(:,5);

figure(1)
clf

plot(X,Y);
grid on; grid minor;
title('Траектория Y(X)');
legend('1 корабль', '2 корабль')
xlabel('X, м');
ylabel('Y, м');

function U = randn_U(c, U0, n)
    U = zeros(2,n);
    U(:,1) = U0;
    
    for m = 1:2
        for i = 1:n
            A = sign(randn);
            %как задать ограничение на максимальный угол поворота?
            if abs(U(m,i) + A*c(m)) > 35
                U(m,i+1) = U(m,i)- A*c(m); %или U(m,i+1) = U(m,i) - тогда выходит на прямую...
            else
                U(m,i+1) = U(m,i) + A*c(m);
            end
        end
    end
    windowSize = 2; 
    b = (1/windowSize)*ones(1,windowSize);
    U = filter(b,1,U);
end

function y = const2struc(Tr, Kr, Td, Kd, P)
    y.Tr = Tr; %постоянная времени в рулевом дифе
    y.Kr = Kr; %коэф перекладки руля
    y.Td = Td; %постоянная времени для функции с углом курса
    y.Kd = Kd; % коэффициент перекладки руля
    y.P = P; %тяга
end
