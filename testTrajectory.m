Ts = 0.01;
N = 500;
x = 1/N/Ts*[1 zeros(1,N-1) -1];




firstBlock = out.simout.signals.values;

time = out.simout.time;


subplot(4,1,1)
plot(time, firstBlock(:,1))

subplot(4,1,2)
plot(time, firstBlock(:,2))

subplot(4,1,3)
plot(time, firstBlock(:,3))

subplot(4,1,4)
plot(time, firstBlock(:,4))