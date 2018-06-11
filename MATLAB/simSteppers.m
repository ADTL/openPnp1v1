

function [t1 t2]=simSteppers()
port_clock = 5e8;
LEN=2^15;
scale = (2^31)/sqrt(LEN);
acc=75;
ticks_mm=51200/2/90;
steps1 = round(ticks_mm * 500)
steps2 = round(ticks_mm * 0.1)
f = steps1/steps2;
period = port_clock/2e5;

[acc_steps , v_steps , period] = calcMovement( steps1, period  ,LEN , acc , port_clock);
[ta1est, tv1est] = calcTtot(scale , toFixed(acc), acc_steps,v_steps , period);
[t1 , v1 , tv1 , a1 , tacc1] = moveStepper(toFixed(acc) , acc_steps ,v_steps , period , scale , port_clock , 0);

%t_tot1-t1(end);


acc = acc/f;
period = period*f;
[acc_steps , v_steps , period] = calcMovement( steps2, period , LEN , acc , port_clock);
%acc_steps = ceil(acc_steps/f);
%v_steps = steps2 - 2*acc_steps;
[ta2est, tv2est] = calcTtot(scale , toFixed(acc), acc_steps, v_steps , period);


terror=0;%round(steps2/(ta1est + tv1est - ta2est - tv2est));
corr = ta1est/ta2est;
acc = acc * corr;
%period = period/sqrt(corr);

[ta2est, tv2est] = calcTtot(scale , toFixed(acc), acc_steps, v_steps , period);
period = (ta1est +tv1est - ta2est)/v_steps;

%t_tot2 = (ta+period*v_steps)/port_clock
[t2 , v2 , tv2 , a2 , tacc2] = moveStepper(toFixed(acc) , acc_steps ,v_steps , period , scale , port_clock , terror);

t1(end) - t2(end)
subplot(2,1,1), plot( t1  , (1:steps1)/ticks_mm , '*-' ,...
                      t2  , (1:steps2)/ticks_mm , '*-' );
grid on
xlabel('Time [s]');
sc=ticks_mm*1000*port_clock;
subplot(2,1,2), plot( tv1  , v1/sc , '*-' ,...
                      tv2  , v2/sc , '*-' ,...
                      tacc1  , a1/sc, tacc2 , a2/sc);
xlabel('Time [s]');
ylabel('v [m/s]');
grid on
end

function  [acc_steps , v_steps , period] = calcMovement( steps , period , LEN , acc , clock)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
scale = (2^31)/sqrt(LEN);
acc_steps = calcAccSteps( scale , acc , steps , period);
dt_min = calc_dt(scale , acc , acc_steps);
t = clock/2e5;
if(dt_min < period)
    dt_min = period;
end
error = dt_min/period-1;
disp(error);
v_steps = steps - 2*acc_steps; 
if(v_steps >0)
   period = dt_min; 
end
  
end


% t(steps) = c * sqrt(steps/a)
%dt(steps) = c/(2*sqrt(steps)*sqrt(a))
% sqrt(steps) = c / (2*sqrt(a) * dt)
% steps = c^2 / (4*a*dt^2)
function acc_steps = calcAccSteps( scale , acc , steps , dt)
    acc_steps= floor(scale^2/(4*acc*dt^2));
    if(acc_steps > floor(steps/2))
        acc_steps = floor(steps/2);
    end
end

function dt = calc_dt(scale , acc , steps)
dt = scale * (sqrt((steps) / acc) - sqrt((steps-1) / acc));
end

function [t , v , tv , a ,ta] = moveStepper(sqrt_inv_a , acc_steps ,v_steps , period ,scale ,port_clock , terror)

%t_tot = scale * sqrt_inv_a*(sqrt((acc_steps-1))+ sqrt((acc_steps))) + period*v_steps;
[ta,tv] = calcTtot(scale , sqrt_inv_a, acc_steps,v_steps , period);
t_tot = ta + tv;

s=0:acc_steps-1;
t1 = scale * sqrt(s) * sqrt_inv_a; % Lookup table
t2 = t1(end)+period : period : t1(end)+period*v_steps;
s=acc_steps-1:-1:0;
t3 = t_tot - scale * sqrt(s) * sqrt_inv_a; % Lookup table
% -------------------------
if(isnan(t2))
    t=([t1 , t3])/port_clock;
else
t=([t1 , t2 , t3])/port_clock;
end
%tk=0*t;
%tk(1:terror:length(t))=1;

%t = floor(t + cumsum(tk))/port_clock;


%t=filter([0.5 0.5], 1,t);

v= 1./(diff(t/port_clock));
ta=t(2:end-1);
tv = conv([0.5 0.5] , t); % calculate the time between t
tv = tv(2:end-1);
a= (diff(v)./diff(tv));
%a=filter(1/500 * ones(500,1) , 1 ,a);
end

function a=toFixed(acc)
a = round(2^32/sqrt(acc))/2^32;
end

function [ta,tv]=calcTtot(scale , sqrt_inv_a, acc_steps, v_steps , period)
ta = scale *sqrt_inv_a*(sqrt(acc_steps)+ sqrt((acc_steps-1)));
if(v_steps == 0)
    tv=0;
else
    tv = period*(v_steps);
end
end