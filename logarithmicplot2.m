%x=logspace(1,3);
%y=exp(x);
%loglog(x,y,'-s')
%grid on

%x=linspace(1, 30, 50);
x=[x 30 200];
plot(x, 10.^x, '-s')

