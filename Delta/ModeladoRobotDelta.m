clc;
clear;

a = 6.5;  %9;
b = 24.0; %18;
h = 2.5;  %6;
r = 9.0;  %12;
x = 0;
y = 0;
z = 0; % Desplazamiento vertical [0,7]

% [,,5]
%
%

p = [x ; y ; z + sqrt(b^2 - (r+a - h)^2)];
c = zeros(3, 3);
%
%   | c1x c2x c3x |
% c=| c1y c2y c3y |
%   | c1z c2z c3z |
%
phi = [0, 120, 240]'*(2*pi)/360;
%ROTMATz(phi(1));% rotamos alrededor de z cero grados (phi_0)
theta = zeros(3, 3);
%
%   | TH11 TH12 TH13 |
% c=| TH21 TH22 TH23 |
%   | TH31 TH32 TH33 |
%

% c(:, 1) = p + [h; 0; 0];
% c(1, 2) = p(1) - abs(h)*cosd(60);
% c(2, 2) = p(2) + abs(h)*sind(60);
% c(3, 2) = p(3);
% c(1, 3) = p(1) - abs(h)*cosd(60);
% c(2, 3) = p(2) - abs(h)*sind(60);
% c(3, 3) = p(3);

%PASO 1 calculamos los puntos C
for i = 1 : 3
    c(:, i) = ROTMATz(phi(i))'*p + [h-r; 0; 0];
end

theta(3, :) = acos(c(2, :)/b);

num = 0;
den = 0;
for i = 1 : 3
    num = c(1, i)^2 + c(2, i)^2 + c(3, i)^2-(a^2 + b^2);
    den = 2*a*b*sin(theta(3, i));
    theta(2, i) = acos(num/den);
end

for i = 1 : 3
    M = a + b*sin(theta(3, i))*cos(theta(2, i));
    N = b*sin(theta(3, i))*sin(theta(2, i));
    theta(1, i) = atan2(-c(1, i)*N + c(3, i)*M, c(3, i)*N + c(1, i)*M);
end

%display(theta);

theta = theta;%*180/pi;

display(sprintf('p=[%2.2f,%2.2f,%2.2f]^T ',p(1),p(2),p(3)));
display(sprintf('TH1=[%2.2f,%2.2f,%2.2f] ',theta(1,1),theta(1,2),theta(1,3)));
display(sprintf('TH2=[%2.2f,%2.2f,%2.2f] ',theta(2,1),theta(2,2),theta(2,3)));
display(sprintf('TH3=[%2.2f,%2.2f,%2.2f] ',theta(3,1),theta(3,2),theta(3,3)));

