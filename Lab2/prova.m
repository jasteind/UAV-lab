
dt=ans.signal2.Time(2)-ans.signal2.Time(1);
I=ans.signal2.Data(1,3)*dt;
Iv(1)=I;
for r=2:length(ans.signal2.Time)-1 
    I=ans.signal2.Data(r,3)*dt;
    Iv(r)=I+Iv(r-1);   
end
figure
plot(Iv)