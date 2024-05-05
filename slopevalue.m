% 기울기 값 구하기_D65

m=(y-0.3290)./(x-0.3127)


% distance 값 구하기

X=(x-Ax).^2 + (y-Ay).^2
d=sqrt(X)

% Linear Expansion

y2=m.*x2-m.*x+y