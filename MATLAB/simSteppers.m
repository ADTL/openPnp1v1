function [t1 t2]=simSteppers()
port_clock = 5e8;
LEN=2^15;
scale = double(int32(inf)+1)/sqrt(LEN);
acc=800; % kvadrat
ticks_mm=12800/90;
steps1 = round(ticks_mm * 100)
steps2 = round(ticks_mm * 10) %Must be less than steps1 NOT SORTED
f = steps1/steps2;
period = port_clock/2e5;

[acc_steps , v_steps , period] = calcMovement( steps1, period  ,LEN , acc , port_clock);
[ta1est, tv1est] = calcTtot(scale , acc, acc_steps, v_steps , period);
[t1 , v1 , tv1 , a1 , tacc1] = moveStepper(toFixed(acc) ,ta1est+ tv1est , acc_steps ,v_steps , period , scale , port_clock);

period = period*f;
acc=acc/f;

if(acc > 250)
    scaledown = acc/250
end

[acc_steps , v_steps , period] = calcMovement( steps2, period , LEN , acc , port_clock);
[ta2est, tv2est] = calcTtot(scale , acc, acc_steps, v_steps , period);
 
%corr = (ta1est+ tv1est)/(ta2est + tv2est)

% disp(1/corr);
% acc = acc / corr^2;
% period = period*corr;
% % 
% [ta2est, tv2est] = calcTtot(scale , acc, acc_steps, v_steps , period);
%  period = (ta1est +tv1est - ta2est)/v_steps;
% t_tot2 = (ta+period*v_steps)/port_clock
 [t2 , v2 , tv2 , a2 , tacc2] = moveStepper(toFixed(acc) ,ta2est+ tv2est , acc_steps ,v_steps , period , scale , port_clock);
%t1(end) - t2(end)
subplot(2,1,1), plot( t1  , (1:steps1)/ticks_mm , '*-' ,...
                      t2  , (1:steps2)/ticks_mm , '*-' );
grid on
xlabel('Time [s]');
ylabel('Distance [mm]');
sc=ticks_mm*1000*port_clock;
subplot(2,1,2), plot( tv1  , v1/sc , '*-' ,...
                      tv2  , v2/sc , '*-' ,...
                      tacc1  , 0*a1/sc, tacc2 , 0*a2/sc);
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
dt = scale/sqrt(acc)  * (sqrt(steps) - sqrt(steps-1));
end

function [t , v , tv , a ,ta] = moveStepper(sqrt_inv_a , t_tot , acc_steps ,v_steps , period ,scale ,port_clock)

%t_tot = scale * sqrt_inv_a*(sqrt((acc_steps-1))+ sqrt((acc_steps))) + period*v_steps;
%[ta,tv] = calcTtot(scale , sqrt_inv_a, acc_steps,v_steps , period);
%t_tot = ta + tv;

s=0:acc_steps-1;
t1 = scale * sqrt(s) * sqrt_inv_a; % Lookup table
if(isempty(t1))
    t2 = linspace(0 , period*v_steps , v_steps);
else
    t2 = t1(end)+period : period : t1(end)+period*v_steps;
end
s=acc_steps-1:-1:0;
t3 = t_tot - scale * sqrt(s) * sqrt_inv_a; % Lookup table
% -------------------------
if(isnan(t2))
    t=floor([t1 , t3])/port_clock;
else
t=floor([t1 , t2 , t3])/port_clock;
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
if( ~isreal(a))
    error("Complex acc");
end
end

function [ta,tv]=calcTtot(scale , acc, acc_steps, v_steps , period)
if(acc_steps<1)
    ta=0;
else
    ta = scale /sqrt(acc)*(sqrt(acc_steps)+ sqrt((acc_steps-1)));
end
if(v_steps == 0)
    tv=0;
else
    tv = period*(v_steps);
end
end