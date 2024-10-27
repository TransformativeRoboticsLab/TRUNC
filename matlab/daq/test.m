clf; close all;
clear; clc;
%d = daqlist("ni")
dq = daq("ni");
dq.Rate = 10000;

ch1 = addinput(dq, "Dev1", "ai0", "Voltage");
ch2 = addinput(dq, "Dev1", "ai4", "Voltage");
ch3 = addinput(dq, "Dev1", "ai2", "Voltage");
ch4 = addinput(dq, "Dev1", "ai6", "Voltage");
ch1.TerminalConfig = "SingleEnded";
ch2.TerminalConfig = "SingleEnded";
ch3.TerminalConfig = "SingleEnded";
ch4.TerminalConfig = "SingleEnded";

%%
figure(1); clf; hold on
data = read(dq, seconds(10), "OutputFormat", "Matrix");
t = 0:(1/10000):(10-(1/10000));
t = t';
plot(t, data(:,1)); hold on;
plot(t, data(:,2)); hold on;
plot(t, data(:,3)); hold on;
plot(t, data(:,4)); hold on;
legend('A','B','C','D');
data(data<2.5) = 0;
data(data>2.5) = 1;
title('Check');

%% Data processing
chA = data(:,1);
chB = data(:,2);
chC = data(:,3);
chD = data(:,4);

count1 = 0;
countarray1 = zeros(100000,1);
count2 = 0;
countarray2 = zeros(100000,1);

for i = 2:100000

	if (chA(i-1,1)==0) && (chA(i,1)==1)
		if (chB(i,1)==1)
			dir1 = 1;
		else 
			dir1 = -1;
		end
	count1 = count1 + dir1;
	countarray1(i,1) = mod(count1, 1024);
	end

	if (chC(i-1,1)==0) && (chC(i,1)==1)
		if (chD(i,1)==1)
			dir2 = -1;
		else 
			dir2 = 1;
		end
	count2 = count2 + dir2;
	countarray2(i,1) = mod(count2, 1024);
	end

end

plot(t, countarray1, t, countarray2)

countarray1 = countarray1(countarray1~=0);
t1 = t(countarray1~=0);
countarray1 = countarray1*360/1024;
ans1 = wrapTo180(countarray1);
countarray2 = countarray2(countarray2~=0);
t2 = t(countarray2~=0);
countarray2 = countarray2*360/1024;
ans2 = wrapTo180(countarray2);

clf; close all;

n= 10000;
subplot(2,1,1)
plot(t1(1:n), ans1(1:n), 'linewidth',2); hold on;
plot(t2(1:n), ans2(1:n),'r--', 'linewidth', 1.5); title('Angular Position of Shaft');
xlabel('time'); ylabel('Angular Position'); legend('Driving Side', 'Driven Side');

subplot(2,1,2);
plot(wrapTo180(ans1(1:n)-ans2(1:n)));
title('Difference Between Angular Positions of Shafts wrapped to (-180,180]');
xlabel('Index'); ylabel('\Delta');

sgtitle('Evaluation of the constant velocity nature of compliant U-Joint');

saveas(gcf, 'shear25mm.png');
filedata = [t1(1:n), ans1(1:n), t2(1:n), ans2(1:n)];
writematrix(filedata,'shear25mm.txt','Delimiter',',');

