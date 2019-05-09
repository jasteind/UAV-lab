for n=1:20
load(['data',num2str(n),'.mat']);
input=inputData.Data;
output=pitchData.Data;

idd=iddata(output,input,100/length(output));

sys=tfest(idd,2)
bode(sys)
hold on
end