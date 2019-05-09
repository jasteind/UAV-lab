clear all; clc
for i=1:20
    %data loading
    load(['data',num2str(i),'.mat']);
    input=inputData.Data;
    output=pitchData.Data;
    %procedure to get right results
    fs=length(output)/100;

    Ni = 2^nextpow2(numel(input));
    No = 2^nextpow2(numel(output));
    %fast fourier transform
    Yi = fft(input-mean(input), Ni)/Ni;
    Yo = fft(output-mean(output), No)/No;
    %definition on frequency domain
    fi = fs/2*linspace(0, 1, Ni/2+1);
    fo = fs/2*linspace(0, 1, No/2+1);
    %correction on the real amplitude
    Yi1 = 2*abs(Yi(1:Ni/2+1));
    Yo1 = 2*abs(Yo(1:No/2+1));
    %collecting results
    [maxi,posr]=max(Yi1);
    [maxo,posf]=max(Yo1);
    transf(i)=maxo/maxi;
    amplitude(i)=20*log10(maxo/maxi);
    pulse(i)=fi(posf)*2*pi; %dimensionally rad/sec
    clear input output
    clear(['data',num2str(i),'.mat'])
end
%plot of the result
figure()
semilogx(pulse,amplitude,'o')
hold on
grid on
PP=spline(pulse,amplitude);
x=[min(pulse):0.1:max(pulse)];
amplitude_spline=ppval(PP,x);
semilogx(x,amplitude_spline)
xlabel('frequency [rad/s]')
ylabel('Amplitude')
legend('experimental points','approximated spline')

% % THIS IS FUCKING DRIVING ME MAD
% beta=nlinfit(pulse,amplitude,@(b,pulse) 20*log10(abs((b(1)*pulse+b(2))./(b(3)*(pulse).^2+b(4)*pulse+b(5)))),[1 1 1 1 1])
% sys=tf(beta(1:2),beta(3:5))
% hold on
% bode(sys)

w0 = 2.9; %2.9
d = 0.68;  %0.68
k = 2.70;  %2.70     

sys=tf([k k],[1 2*d*w0 w0^2])
hold on
margin(sys)


