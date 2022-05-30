classdef ship1
    properties
        F
        kof
        %kof.Tr  %постоянная времени в рулевом дифе
        %kof.Kr %коэф перекладки руля
        %kof.Td %постоянная времени для функции с углом курса
        %kof.Kd %коэф перекладки руля
        %kof.P ; %тяга
        U %сигнал управления рулем
    end
    methods
        function obj = ship1(F0,koff, U) %конструктор
            obj.F = F0;
            obj.kof.Tr = koff.Tr;
            obj.kof.Kr = koff.Kr;
            obj.kof.Td = koff.Td;
            obj.kof.Kd = koff.Kd;
            obj.kof.P = koff.P;
            obj.U = deg2rad(U);
        end
        function dF = fnc_dF(obj,t,y,i) %преращения
            dF(1) = (obj.kof.Kd * obj.U(i)-y(1))/obj.kof.Td; %рулевой угол
            dF(2) = y(3); %угол курса
            dF(3) = (obj.kof.Kr * y(1) - y(3))/obj.kof.Tr; %угловая скорость
            dF(4) = obj.kof.P * cos(y(1)) * cos(y(2)) - obj.kof.P * sin(y(1)) * sin(y(2)); %х
            dF(5) = obj.kof.P * cos(y(1)) * sin(y(2)) + obj.kof.P * sin(y(1)) * cos(y(2)); %у
        end
    end
end