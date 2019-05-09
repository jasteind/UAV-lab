%% Pitch and Roll from Accelerometers

time=ans.signal1.Time;

x_acc=ans.signal1.Data(:,1);
y_acc=ans.signal1.Data(:,2);
z_acc=ans.signal1.Data(:,3);

a=find(x_acc==0);
b=find(y_acc==0);
c=find(z_acc==0);
x_acc=x_acc(a(end)+1:end);
y_acc=y_acc(b(end)+1:end);
z_acc=z_acc(c(end)+1:end);

phi_meas = atand (y_acc./z_acc);
theta_meas =  atand (-x_acc.*cosd(phi_meas)./z_acc);
